require("dotenv").config();
const fs = require("fs");
const path = require("path");
const net = require("net");
const express = require("express");
const modbus = require("modbus-tcp");

const MODBUS_PORT = Number(process.env.MODBUS_PORT || 502);
const MODBUS_HOST = process.env.MODBUS_HOST || "0.0.0.0";
const WEB_PORT = Number(process.env.WEB_PORT || 8081);
const DEFAULT_METER_COUNT = Number(process.env.METER_COUNT || 3);
const MAX_METERS = Number(process.env.MAX_METERS || 2000);
// Modbus unit IDs are 1-247; meters beyond that spill onto the next TCP port.
const UNITS_PER_PORT = Math.min(Math.max(Number(process.env.UNITS_PER_PORT || 247), 1), 247);
const TICK_MS = 100;

// Meter N (1-based) -> Modbus endpoint
function endpointOf(id) {
  return {
    port: MODBUS_PORT + Math.floor((id - 1) / UNITS_PER_PORT),
    unitId: ((id - 1) % UNITS_PER_PORT) + 1
  };
}

// IFC050 status bit positions in the u32 (ABCD): u32 bit = (3 - byte) * 8 + bit-in-byte
// Manual-toggleable bits per meter; flowSign / emptyPipeF / device error are auto-derived.
const SENSOR_BIT_POS = {
  fatalError: 31,      // byte 0 bit 7: Fatal error in sensor electronic
  appError: 30,        // byte 0 bit 6: Application error
  outOfSpec: 29,       // byte 0 bit 5: Out of specification
  flowOverRange: 25,   // byte 0 bit 1: Flow over range
  flowSign: 20,        // byte 1 bit 4: Flow sign (auto: negative flow)
  emptyPipeF: 18,      // byte 1 bit 2: Empty pipe (F) (auto from emptyPipe flag)
  emptyPipeS: 15,      // byte 2 bit 7: Empty pipe (S)
  emptyPipeI: 12,      // byte 2 bit 4: Empty pipe (I)
  coilTemp: 4,         // byte 3 bit 4: Coil temperature out of range
  gainError: 1         // byte 3 bit 1: Gain error
};

const DEVICE_BIT_POS = {
  error: 31,           // byte 0 bit 7: Error in device (auto from error flag)
  appError: 30,        // byte 0 bit 6: Application error
  uncertain: 29,       // byte 0 bit 5: Uncertain measurement
  checks: 28           // byte 0 bit 4: Checks in progress
};

const SENSOR_BIT_TABLE = [
  { key: "fatalError", pos: 31, bit: "B0.7", message: "Fatal error in sensor electronic", description: "Error or failure of the sensor electronic, parameter or hardware error, this cannot be used any longer.", severity: "error" },
  { key: "appError", pos: 30, bit: "B0.6", message: "Application error", description: "Application error has occurred, the measuring device is however ok, the measured values are not valid", severity: "warning" },
  { key: "outOfSpec", pos: 29, bit: "B0.5", message: "Out of specification", description: "Maintenance required, measured value restrictedly usable", severity: "warning" },
  { key: "flowOverRange", pos: 25, bit: "B0.1", message: "Flow over range", description: "Over range, the measured values are limited by the filter setting.", severity: "warning" },
  { key: "flowSign", pos: 20, bit: "B1.4", message: "Flow sign", description: "1 = negative flow", severity: "info" },
  { key: "emptyPipeF", pos: 18, bit: "B1.2", message: "Empty pipe (F)", description: "One or both measuring electrodes have no contact for fluidity, flow measured value is set to zero, no flow measurement possible", severity: "warning" },
  { key: "emptyPipeS", pos: 15, bit: "B2.7", message: "Empty pipe (S)", description: "One or both measuring electrodes have no contact for fluidity, flow measured value is set to zero, no flow measurement possible", severity: "warning" },
  { key: "emptyPipeI", pos: 12, bit: "B2.4", message: "Empty pipe (I)", description: "One or both measuring electrodes have no contact for fluidity, flow measured value is set to zero, no flow measurement possible", severity: "warning" },
  { key: "coilTemp", pos: 4, bit: "B3.4", message: "Coil temperature out of range", description: "The maximum coil temperature is exceeded, no message if coil bridged or broken.", severity: "warning" },
  { key: "gainError", pos: 1, bit: "B3.1", message: "Gain error", description: "The preamplifier gain does not correspond to the calibrated value, calibration check, flow measured values are provided further on", severity: "warning" }
];

const DEVICE_BIT_TABLE = [
  { key: "error", pos: 31, bit: "B0.7", message: "Error in device", description: "Error or failure of the complete device, parameter or hardware error, device cannot be used any longer.", severity: "error" },
  { key: "appError", pos: 30, bit: "B0.6", message: "Application error", description: "Application-contingent error of the complete device, the device is however in order", severity: "warning" },
  { key: "uncertain", pos: 29, bit: "B0.5", message: "Uncertain measurement", description: "Maintenance of the device necessary, measured values only conditionally usable.", severity: "warning" },
  { key: "checks", pos: 28, bit: "B0.4", message: "Checks in progress", description: "Test run of the device, measured values can be simulated measured values or be set to a fixed value.", severity: "info" }
];

function decodeStatus(value, table) {
  const v = Number(value) >>> 0;
  const bytes = [(v >>> 24) & 0xff, (v >>> 16) & 0xff, (v >>> 8) & 0xff, v & 0xff];
  const active = [];
  const unknown = [];
  for (let b = 0; b < 4; b++) {
    for (let bit = 7; bit >= 0; bit--) {
      if (!(bytes[b] & (1 << bit))) continue;
      const pos = (3 - b) * 8 + bit;
      const def = table.find(t => t.pos === pos);
      if (def) active.push({ bit: `B${b}.${bit}`, message: def.message, description: def.description, severity: def.severity });
      else unknown.push(`B${b}.${bit}`);
    }
  }
  let severity = "ok";
  if (active.some(a => a.severity === "error")) severity = "error";
  else if (active.some(a => a.severity === "warning")) severity = "warning";
  else if (active.length || unknown.length) severity = "info";
  return {
    value: v,
    hex: "0x" + v.toString(16).padStart(8, "0").toUpperCase(),
    bits: bytes.map(b => b.toString(2).padStart(8, "0")).join(" "),
    active,
    unknown,
    severity
  };
}

function writeFloatBE(arr, idx, val) {
  const b = Buffer.alloc(4);
  b.writeFloatBE(val, 0);
  arr[idx] = b.readUInt16BE(0);
  arr[idx + 1] = b.readUInt16BE(2);
}

function readFloatBE(arr, idx) {
  const b = Buffer.alloc(4);
  b.writeUInt16BE(arr[idx] & 0xffff, 0);
  b.writeUInt16BE(arr[idx + 1] & 0xffff, 2);
  return b.readFloatBE(0);
}

function writeDoubleBE(arr, idx, val) {
  const b = Buffer.alloc(8);
  b.writeDoubleBE(val, 0);
  for (let i = 0; i < 4; i++) arr[idx + i] = b.readUInt16BE(i * 2);
}

function normInput(addr) {
  if (addr >= 30000 && addr <= 38998) return addr - 30000;
  if (addr >= 20000 && addr <= 20998) return 9000 + (addr - 20000);
  return addr;
}

function normHolding(addr) {
  if (addr >= 40000 && addr <= 49998) return addr - 40000;
  return addr;
}

function makeMeter(id) {
  const holding = new Uint16Array(10000);
  holding[2000] = 0;
  holding[2001] = 0;
  holding[2002] = 0;
  writeFloatBE(holding, 3000, 0.0);
  writeFloatBE(holding, 3002, 12.0);
  writeFloatBE(holding, 3004, 3.0);
  writeFloatBE(holding, 3006, 0.05);
  writeFloatBE(holding, 3008, 12.0);
  writeFloatBE(holding, 3010, 0.05);
  writeFloatBE(holding, 3012, 0.02);
  writeFloatBE(holding, 3014, 0.0);
  writeFloatBE(holding, 3016, 0.0);
  writeFloatBE(holding, 3018, 1.0);
  writeFloatBE(holding, 3020, 0.0);
  writeFloatBE(holding, 3022, 0.0);

  return {
    id,
    ...endpointOf(id),
    holding,
    targetSpeed: 0.0,
    filtSpeed: 0.0,
    vary: false,
    varyPct: 5,
    varyOffset: 0,
    varyNext: 0,
    dispSpeed: 0.0,
    dispVol: 0.0,
    dispMass: 0.0,
    pipeDiamMm: 100,
    density: 1000,
    operatingTime: 0,
    counter1: 0,
    counter2: 0,
    c1run: false,
    c2run: false,
    emptyPipe: false,
    error: false,
    sensorBits: {},
    deviceBits: {},
    statusSensor: 0,
    statusDevice: 0,
    wordOrder: "ABCD",
    logs: []
  };
}

const serverLog = [];
const MAX_SERVER_LOGS = 300;

function slog(line) {
  serverLog.push(`[${ts()}] ${line}`);
  if (serverLog.length > MAX_SERVER_LOGS) serverLog.splice(0, serverLog.length - MAX_SERVER_LOGS);
  console.log(line);
}

const MAX_LOGS = 300;

function ts() {
  const d = new Date();
  return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0");
}

function logRange(m, fc, label, from, to) {
  const range = to > from ? `${from}-${to}` : `${from}`;
  const qty = to - from + 1;
  m.logs.push(`[${ts()}] ${fc} ${label} ${range} (${qty} reg${qty > 1 ? "s" : ""})`);
  if (m.logs.length > MAX_LOGS) m.logs.splice(0, m.logs.length - MAX_LOGS);
}

function logWrite(m, fc, label, addr, value) {
  m.logs.push(`[${ts()}] ${fc} ${label} ${addr} <- ${value}`);
  if (m.logs.length > MAX_LOGS) m.logs.splice(0, m.logs.length - MAX_LOGS);
}

const meters = [];

const SAVE_FILE = process.env.STATE_FILE || path.join(__dirname, "simulator-state.json");

// Holding registers are stored sparsely ({index: value}, non-zero only) so the
// state file stays small with hundreds of meters. Older files with a full array still load.
function sparseHolding(holding) {
  const out = {};
  for (let j = 0; j < holding.length; j++) if (holding[j]) out[j] = holding[j];
  return out;
}

// Settings a user configures; copied by "copy settings" and persisted.
function meterSettings(m) {
  return {
    targetSpeed: m.targetSpeed,
    vary: m.vary,
    varyPct: m.varyPct,
    pipeDiamMm: m.pipeDiamMm,
    density: m.density,
    emptyPipe: m.emptyPipe,
    error: m.error,
    sensorBits: { ...m.sensorBits },
    deviceBits: { ...m.deviceBits },
    wordOrder: m.wordOrder,
    holding: sparseHolding(m.holding)
  };
}

function applySettings(m, d) {
  if (typeof d.targetSpeed === "number") m.targetSpeed = d.targetSpeed;
  if (typeof d.vary === "boolean") m.vary = d.vary;
  if (typeof d.varyPct === "number") m.varyPct = d.varyPct;
  if (typeof d.pipeDiamMm === "number") m.pipeDiamMm = d.pipeDiamMm;
  if (typeof d.density === "number") m.density = d.density;
  if (typeof d.emptyPipe === "boolean") m.emptyPipe = d.emptyPipe;
  if (typeof d.error === "boolean") m.error = d.error;
  if (d.sensorBits && typeof d.sensorBits === "object") {
    for (const k of Object.keys(SENSOR_BIT_POS)) m.sensorBits[k] = !!d.sensorBits[k];
  }
  if (d.deviceBits && typeof d.deviceBits === "object") {
    for (const k of Object.keys(DEVICE_BIT_POS)) m.deviceBits[k] = !!d.deviceBits[k];
  }
  if (typeof d.wordOrder === "string") m.wordOrder = d.wordOrder;
  if (Array.isArray(d.holding)) {
    const n = Math.min(d.holding.length, m.holding.length);
    for (let j = 0; j < n; j++) m.holding[j] = d.holding[j] & 0xffff;
  } else if (d.holding && typeof d.holding === "object") {
    m.holding.fill(0);
    for (const [j, v] of Object.entries(d.holding)) {
      const idx = Number(j);
      if (idx >= 0 && idx < m.holding.length) m.holding[idx] = v & 0xffff;
    }
  }
}

function saveState() {
  try {
    const data = meters.map(m => ({
      ...meterSettings(m),
      c1run: m.c1run,
      c2run: m.c2run,
      counter1: m.counter1,
      counter2: m.counter2,
      operatingTime: m.operatingTime
    }));
    fs.writeFileSync(SAVE_FILE, JSON.stringify(data));
  } catch (e) {
    console.error("state save failed:", e.message);
  }
}

// The meter count comes from the saved state if there is one, else METER_COUNT.
function loadState() {
  let data = null;
  try {
    if (fs.existsSync(SAVE_FILE)) {
      const parsed = JSON.parse(fs.readFileSync(SAVE_FILE, "utf8"));
      if (Array.isArray(parsed) && parsed.length) data = parsed;
    }
  } catch (e) {
    console.error("state load failed:", e.message);
  }
  const count = clampCount(data ? data.length : DEFAULT_METER_COUNT);
  for (let i = 1; i <= count; i++) meters.push(makeMeter(i));
  if (!data) return;
  data.slice(0, count).forEach((d, i) => {
    const m = meters[i];
    if (!d) return;
    applySettings(m, d);
    if (typeof d.c1run === "boolean") m.c1run = d.c1run;
    if (typeof d.c2run === "boolean") m.c2run = d.c2run;
    if (typeof d.counter1 === "number") m.counter1 = d.counter1;
    if (typeof d.counter2 === "number") m.counter2 = d.counter2;
    if (typeof d.operatingTime === "number") m.operatingTime = d.operatingTime;
  });
  console.log(`restored ${count} meters from ${SAVE_FILE}`);
}

function clampCount(n) {
  n = Math.floor(Number(n));
  if (!Number.isFinite(n) || n < 1) return 1;
  return Math.min(n, MAX_METERS);
}

loadState();
process.on("SIGINT", () => { saveState(); process.exit(0); });
process.on("SIGTERM", () => { saveState(); process.exit(0); });
process.on("exit", () => saveState());
setInterval(saveState, 5000).unref();

function computeOutputs(m) {
  const limLo = readFloatBE(m.holding, 3000);
  const limHi = readFloatBE(m.holding, 3002);
  const cutoff = readFloatBE(m.holding, 3012);
  const zeroPoint = readFloatBE(m.holding, 3016);
  const dir = m.holding[2000] === 1 ? -1 : 1;

  let eff = m.filtSpeed;
  if (m.emptyPipe || m.error) eff = 0;
  if (Math.abs(eff) < cutoff) eff = 0;
  if (Math.abs(eff) < limLo) eff = 0;
  if (Math.abs(eff) > limHi) eff = Math.sign(eff) * limHi;

  m.dispSpeed = dir * eff + (eff !== 0 ? zeroPoint : 0);
  const area = Math.PI * Math.pow(m.pipeDiamMm / 1000, 2) / 4;
  m.dispVol = m.dispSpeed * area;
  m.dispMass = m.dispVol * m.density;

  // IFC050 manual bit layout, u32 big-endian (ABCD): u32 bit = (3 - byte) * 8 + bit-in-byte
  let ss = 0;
  for (const [key, pos] of Object.entries(SENSOR_BIT_POS)) {
    if (key === "flowSign" || key === "emptyPipeF") continue;
    if (m.sensorBits[key]) ss |= (1 << pos);
  }
  if (m.emptyPipe) ss |= (1 << SENSOR_BIT_POS.emptyPipeF);
  if (m.dispSpeed < 0) ss |= (1 << SENSOR_BIT_POS.flowSign);
  if (limHi > 0 && Math.abs(m.filtSpeed) > limHi) ss |= (1 << SENSOR_BIT_POS.flowOverRange);
  m.statusSensor = ss >>> 0;

  let sd = 0;
  for (const [key, pos] of Object.entries(DEVICE_BIT_POS)) {
    if (key === "error") continue;
    if (m.deviceBits[key]) sd |= (1 << pos);
  }
  if (m.error) sd |= (1 << DEVICE_BIT_POS.error);
  m.statusDevice = sd >>> 0;
}

let lastTick = Date.now();
setInterval(() => {
  const now = Date.now();
  const dt = (now - lastTick) / 1000;
  lastTick = now;
  for (const m of meters) {
    const tau = Math.max(readFloatBE(m.holding, 3004), 0.05);
    let target = m.targetSpeed;
    if (m.vary) {
      if (now >= m.varyNext) {
        m.varyOffset = (Math.random() * 2 - 1) * (m.varyPct / 100);
        m.varyNext = now + 2000;
      }
      target *= 1 + m.varyOffset;
    } else {
      m.varyOffset = 0;
    }
    m.filtSpeed += (target - m.filtSpeed) * Math.min(dt / tau, 1);
    computeOutputs(m);
    m.operatingTime += dt;
    if (m.c1run) m.counter1 += m.dispVol * dt;
    if (m.c2run) m.counter2 += m.dispVol * dt;
  }
}, TICK_MS);

function permuteElem(arr, start, nRegs, order) {
  if (order === "ABCD") return;
  const tmp = [];
  for (let i = 0; i < nRegs; i++) tmp.push(arr[start + i]);
  const bswap = (v) => ((v & 0xff) << 8) | (v >> 8);
  if (order === "BADC") {
    for (let i = 0; i < nRegs; i++) arr[start + i] = bswap(tmp[i]);
  } else if (order === "CDAB") {
    for (let g = 0; g + 1 < nRegs; g += 2) {
      arr[start + g] = tmp[g + 1];
      arr[start + g + 1] = tmp[g];
    }
  } else if (order === "DCBA") {
    for (let i = 0; i < nRegs; i++) arr[start + i] = bswap(tmp[nRegs - 1 - i]);
  }
}

function inputRegisterBlock(m) {
  const r = new Uint16Array(20);
  writeFloatBE(r, 0, m.dispSpeed);
  writeFloatBE(r, 2, m.dispVol);
  writeFloatBE(r, 4, m.dispMass);
  writeFloatBE(r, 6, m.operatingTime);
  writeDoubleBE(r, 8, m.counter1);
  writeDoubleBE(r, 12, m.counter2);
  r[16] = (m.statusSensor >>> 16) & 0xffff;
  r[17] = m.statusSensor & 0xffff;
  r[18] = (m.statusDevice >>> 16) & 0xffff;
  r[19] = m.statusDevice & 0xffff;
  permuteElem(r, 0, 2, m.wordOrder);
  permuteElem(r, 2, 2, m.wordOrder);
  permuteElem(r, 4, 2, m.wordOrder);
  permuteElem(r, 6, 2, m.wordOrder);
  permuteElem(r, 8, 4, m.wordOrder);
  permuteElem(r, 12, 4, m.wordOrder);
  return r;
}

function getCoilState(m, addr) {
  switch (addr) {
    case 3000: return m.c1run;
    case 3001: return m.c2run;
    case 3003: return false;
    case 3004: return false;
    default: return false;
  }
}

function setCoilState(m, addr, value) {
  switch (addr) {
    case 3000: m.c1run = !!value; break;
    case 3001: m.c2run = !!value; break;
    case 3003: if (value) m.counter1 = 0; break;
    case 3004: if (value) m.counter2 = 0; break;
    default: break;
  }
}

function meterOrNull(port, unitID) {
  if (!(unitID >= 1 && unitID <= UNITS_PER_PORT)) return null;
  return meters[(port - MODBUS_PORT) * UNITS_PER_PORT + unitID - 1] || null;
}

// Unit IDs served on a port, for log messages
function unitRange(port) {
  const first = (port - MODBUS_PORT) * UNITS_PER_PORT + 1;
  const n = Math.max(0, Math.min(UNITS_PER_PORT, meters.length - first + 1));
  return n ? `1-${n}` : "none";
}

function regBuf(v) {
  const b = Buffer.alloc(2);
  b.writeUInt16BE(v & 0xffff, 0);
  return b;
}

function zeroRegs(n) {
  const out = [];
  for (let i = 0; i < n; i++) out.push(regBuf(0));
  return out;
}

function zeroBits(n) {
  return new Array(n).fill(0);
}

function createModbusServer(port) {
  const reject = (fc, unitId, what) =>
    slog(`REJECTED ${fc} port=${port} unit=${unitId} ${what}: unknown unit ID (port ${port} serves units ${unitRange(port)})`);

  const server = net.createServer((socket) => {
    const peer = `${socket.remoteAddress}:${socket.remotePort}`;
    server.sockets.add(socket);
    slog(`client connected: ${peer} -> port ${port}`);
    socket.on("close", () => { server.sockets.delete(socket); slog(`client disconnected: ${peer} (port ${port})`); });
    socket.on("error", (e) => slog(`client ${peer} socket error: ${e.message}`));

    const s = new modbus.Server();
    s.on("error", () => {});
    s.pipe(socket);

    s.on("read-input-registers", (from, to, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      if (!m) {
        reject("FC04", data.unitId, `${from}-${to}`);
        return reply(null, zeroRegs(to - from + 1));
      }
      logRange(m, "FC04", "InputReg", from, to);
      const block = inputRegisterBlock(m);
      const out = [];
      for (let a = from; a <= to; a++) {
        const n = normInput(a);
        out.push(regBuf(n >= 0 && n < block.length ? block[n] : 0));
      }
      reply(null, out);
    });

    s.on("read-holding-registers", (from, to, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      if (!m) {
        reject("FC03", data.unitId, `${from}-${to}`);
        return reply(null, zeroRegs(to - from + 1));
      }
      logRange(m, "FC03", "HoldingReg", from, to);
      const out = [];
      for (let a = from; a <= to; a++) {
        const n = normHolding(a);
        out.push(regBuf(n >= 0 && n < m.holding.length ? m.holding[n] : 0));
      }
      reply(null, out);
    });

    const coilRead = (fc) => (from, to, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      if (!m) {
        reject(fc, data.unitId, `${from}-${to}`);
        return reply(null, zeroBits(to - from + 1));
      }
      logRange(m, fc, "Coil", from, to);
      const out = [];
      for (let a = from; a <= to; a++) out.push(getCoilState(m, a) ? 1 : 0);
      reply(null, out);
    };
    s.on("read-coils", coilRead("FC01"));
    s.on("read-discrete-inputs", coilRead("FC02"));

    s.on("write-single-coil", (address, value, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      const v = value[0] === 0xff;
      if (!m) {
        reject("FC05", data.unitId, `addr=${address}`);
        return reply(null);
      }
      setCoilState(m, address, v);
      logWrite(m, "FC05", "Coil", address, v ? 1 : 0);
      reply(null);
    });

    s.on("write-multiple-coils", (from, to, items, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      if (!m) {
        reject("FC15", data.unitId, `${from}-${to}`);
        return reply(null);
      }
      items.forEach((bit, i) => {
        setCoilState(m, from + i, !!bit);
        logWrite(m, "FC15", "Coil", from + i, bit);
      });
      reply(null);
    });

    s.on("write-single-register", (address, value, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      const v = value.readUInt16BE(0);
      if (!m) {
        reject("FC06", data.unitId, `addr=${address}`);
        return reply(null);
      }
      const n = normHolding(address);
      if (n >= 0 && n < m.holding.length) m.holding[n] = v;
      logWrite(m, "FC06", "HoldingReg", address, v);
      reply(null);
    });

    s.on("write-multiple-registers", (from, to, items, reply, data) => {
      const m = meterOrNull(port, data.unitId);
      if (!m) {
        reject("FC16", data.unitId, `${from}-${to}`);
        return reply(null);
      }
      items.forEach((buf, i) => {
        const n = normHolding(from + i);
        const v = buf.readUInt16BE(0);
        if (n >= 0 && n < m.holding.length) m.holding[n] = v;
        logWrite(m, "FC16", "HoldingReg", from + i, v);
      });
      reply(null);
    });

    s.on("data", (d) => {
      slog(`unsupported function code '${d.functionCode}' from port=${port} unit=${d.unitId}`);
    });
  });
  server.sockets = new Set();
  return server;
}

// One TCP listener per block of UNITS_PER_PORT meters: 502 for 1-247, 503 for 248-494, ...
const mbServers = new Map();

function syncModbusPorts() {
  const needed = Math.ceil(meters.length / UNITS_PER_PORT);
  for (let k = 0; k < needed; k++) {
    const port = MODBUS_PORT + k;
    if (mbServers.has(port)) continue;
    const server = createModbusServer(port);
    mbServers.set(port, server);
    server.on("error", (e) => {
      slog(`Modbus server error on port ${port}: ${e.message}`);
      if (port === MODBUS_PORT) process.exit(1);
      mbServers.delete(port);
    });
    server.listen(port, MODBUS_HOST, () => {
      slog(`Modbus TCP listening on ${MODBUS_HOST}:${port} (unit IDs ${unitRange(port)})`);
    });
  }
  for (const [port, server] of mbServers) {
    if (port - MODBUS_PORT < needed) continue;
    mbServers.delete(port);
    server.close();
    for (const sock of server.sockets) sock.destroy();
    slog(`Modbus TCP port ${port} closed (no meters left on it)`);
  }
}

function setMeterCount(n) {
  const count = clampCount(n);
  while (meters.length < count) meters.push(makeMeter(meters.length + 1));
  if (meters.length > count) meters.length = count;
  syncModbusPorts();
  saveState();
  slog(`meter count set to ${count}`);
}

syncModbusPorts();

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, "public")));

function meterState(m, withLogs = true) {
  return {
    id: m.id,
    port: m.port,
    unitId: m.unitId,
    targetSpeed: m.targetSpeed,
    vary: m.vary,
    varyPct: m.varyPct,
    flowSpeed: m.dispSpeed,
    volumeFlow: m.dispVol,
    volumeFlowM3h: m.dispVol * 3600,
    massFlow: m.dispMass,
    operatingTime: m.operatingTime,
    counter1: m.counter1,
    counter2: m.counter2,
    c1run: m.c1run,
    c2run: m.c2run,
    emptyPipe: m.emptyPipe,
    error: m.error,
    pipeDiamMm: m.pipeDiamMm,
    density: m.density,
    statusSensor: m.statusSensor,
    statusDevice: m.statusDevice,
    statusSensorDecoded: decodeStatus(m.statusSensor, SENSOR_BIT_TABLE),
    statusDeviceDecoded: decodeStatus(m.statusDevice, DEVICE_BIT_TABLE),
    sensorBits: m.sensorBits,
    deviceBits: m.deviceBits,
    wordOrder: m.wordOrder,
    ...(withLogs ? { logs: m.logs } : {}),
    holding: {
      flowDirection: m.holding[2000],
      pulseFilter: m.holding[2001],
      emptyPipeMode: m.holding[2002],
      limitationLow: readFloatBE(m.holding, 3000),
      limitationHigh: readFloatBE(m.holding, 3002),
      timeConstant: readFloatBE(m.holding, 3004),
      pulseWidth: readFloatBE(m.holding, 3006),
      pulseLimitation: readFloatBE(m.holding, 3008),
      pulseWidthAuto: readFloatBE(m.holding, 3010),
      lowFlowCutoff: readFloatBE(m.holding, 3012),
      limitEmptyPipe: readFloatBE(m.holding, 3014),
      zeroPoint: readFloatBE(m.holding, 3016),
      electrodeFactor: readFloatBE(m.holding, 3018),
      conductivityCal1: readFloatBE(m.holding, 3020),
      conductivityCal2: readFloatBE(m.holding, 3022)
    }
  };
}

function worstSeverity(m) {
  const s = [decodeStatus(m.statusSensor, SENSOR_BIT_TABLE).severity, decodeStatus(m.statusDevice, DEVICE_BIT_TABLE).severity];
  if (s.includes("error")) return "error";
  if (s.includes("warning")) return "warning";
  return "ok";
}

// Compact per-meter row for the overview table
function meterSummary(m) {
  return {
    id: m.id,
    port: m.port,
    unitId: m.unitId,
    targetSpeed: m.targetSpeed,
    flowSpeed: m.dispSpeed,
    volumeFlowM3h: m.dispVol * 3600,
    vary: m.vary,
    emptyPipe: m.emptyPipe,
    error: m.error,
    severity: worstSeverity(m),
    lastActivity: m.logs.length ? m.logs[m.logs.length - 1].slice(1, 13) : null
  };
}

function config() {
  const ports = [];
  for (const port of [...mbServers.keys()].sort((a, b) => a - b)) ports.push({ port, units: unitRange(port) });
  return { count: meters.length, maxMeters: MAX_METERS, unitsPerPort: UNITS_PER_PORT, basePort: MODBUS_PORT, ports };
}

// Parse "1-10,15,20-25" (or "all") into meter ids, clamped to existing meters
function parseIds(spec) {
  const ids = new Set();
  for (const part of String(spec || "").split(",")) {
    const t = part.trim();
    if (!t) continue;
    if (t.toLowerCase() === "all") { meters.forEach(m => ids.add(m.id)); continue; }
    const r = t.match(/^(\d+)\s*-\s*(\d+)$/);
    const [a, b] = r ? [Number(r[1]), Number(r[2])] : [Number(t), Number(t)];
    if (!Number.isInteger(a) || !Number.isInteger(b)) continue;
    for (let i = Math.max(1, Math.min(a, b)); i <= Math.min(meters.length, Math.max(a, b)); i++) ids.add(i);
  }
  return [...ids];
}

// GET /api/state            -> all meters, full state without logs (used by verify scripts)
// GET /api/state?summary=1  -> compact rows for the UI table
// &detail=1,5,7             -> full state incl. logs for those meters
app.get("/api/state", (req, res) => {
  const out = { config: config(), serverLog };
  out.meters = req.query.summary ? meters.map(meterSummary) : meters.map(m => meterState(m, false));
  if (req.query.detail) out.detail = parseIds(req.query.detail).map(id => meterState(meters[id - 1]));
  res.json(out);
});

app.get("/api/meter/:id", (req, res) => {
  const m = meters[Number(req.params.id) - 1];
  if (!m) return res.status(404).json({ error: "unknown meter" });
  res.json(meterState(m));
});

app.post("/api/meters/count", (req, res) => {
  const n = Number(req.body && req.body.count);
  if (!Number.isFinite(n) || n < 1) return res.status(400).json({ error: "count must be >= 1" });
  setMeterCount(n);
  res.json(config());
});

// Copy one meter's settings (setpoint, vary, geometry, word order, fault flags,
// status bits, holding registers) onto other meters. Counters are not copied.
app.post("/api/meters/copy", (req, res) => {
  const src = meters[Number(req.body && req.body.from) - 1];
  if (!src) return res.status(404).json({ error: "unknown source meter" });
  const targets = parseIds(req.body.to).filter(id => id !== src.id);
  const settings = meterSettings(src);
  for (const id of targets) applySettings(meters[id - 1], settings);
  saveState();
  res.json({ copied: targets.length });
});

app.post("/api/meter/:id", (req, res) => {
  const m = meters[Number(req.params.id) - 1];
  if (!m) return res.status(404).json({ error: "unknown meter" });
  const b = req.body || {};
  if (typeof b.flowSpeed === "number") m.targetSpeed = b.flowSpeed;
  if (typeof b.vary === "boolean") m.vary = b.vary;
  if (typeof b.varyPct === "number" && b.varyPct >= 0 && b.varyPct <= 100) m.varyPct = b.varyPct;
  if (typeof b.emptyPipe === "boolean") m.emptyPipe = b.emptyPipe;
  if (typeof b.error === "boolean") m.error = b.error;
  if (b.sensorBits && typeof b.sensorBits === "object") {
    for (const [k, v] of Object.entries(b.sensorBits)) {
      if (k === "flowSign" || k === "emptyPipeF") continue; // auto-derived
      if (k in SENSOR_BIT_POS) m.sensorBits[k] = !!v;
    }
  }
  if (b.deviceBits && typeof b.deviceBits === "object") {
    for (const [k, v] of Object.entries(b.deviceBits)) {
      if (k === "error") continue; // auto-derived
      if (k in DEVICE_BIT_POS) m.deviceBits[k] = !!v;
    }
  }
  if (typeof b.pipeDiamMm === "number" && b.pipeDiamMm > 0) m.pipeDiamMm = b.pipeDiamMm;
  if (typeof b.density === "number" && b.density > 0) m.density = b.density;
  if (typeof b.wordOrder === "string" && ["ABCD", "BADC", "CDAB", "DCBA"].includes(b.wordOrder)) m.wordOrder = b.wordOrder;
  res.json(meterState(m));
});

app.post("/api/meter/:id/counter/:n", (req, res) => {
  const m = meters[Number(req.params.id) - 1];
  if (!m) return res.status(404).json({ error: "unknown meter" });
  const n = Number(req.params.n);
  const action = (req.body && req.body.action) || "";
  if (n === 1) {
    if (action === "start") m.c1run = true;
    else if (action === "stop") m.c1run = false;
    else if (action === "reset") m.counter1 = 0;
    else return res.status(400).json({ error: "bad action" });
  } else if (n === 2) {
    if (action === "start") m.c2run = true;
    else if (action === "stop") m.c2run = false;
    else if (action === "reset") m.counter2 = 0;
    else return res.status(400).json({ error: "bad action" });
  } else return res.status(404).json({ error: "unknown counter" });
  res.json(meterState(m));
});

const HOLDING_FLOAT_ADDRS = [3000, 3002, 3004, 3006, 3008, 3010, 3012, 3014, 3016, 3018, 3020, 3022];
const HOLDING_BYTE_ADDRS = [2000, 2001, 2002];

app.post("/api/meter/:id/holding", (req, res) => {
  const m = meters[Number(req.params.id) - 1];
  if (!m) return res.status(404).json({ error: "unknown meter" });
  let addr = Number(req.body && req.body.addr);
  const value = Number(req.body && req.body.value);
  if (!Number.isFinite(addr) || !Number.isFinite(value)) return res.status(400).json({ error: "addr/value required" });
  if (addr >= 42000) addr = 2000 + (addr - 42000);
  else if (addr >= 43000) addr = 3000 + (addr - 43000);
  else if (addr >= 40000) addr -= 40000;
  if (HOLDING_BYTE_ADDRS.includes(addr)) {
    m.holding[addr] = value & 0xffff;
  } else if (HOLDING_FLOAT_ADDRS.includes(addr)) {
    writeFloatBE(m.holding, addr, value);
  } else {
    return res.status(400).json({ error: "unsupported holding address " + addr });
  }
  res.json(meterState(m));
});

app.listen(WEB_PORT, () => {
  console.log(`Web UI: http://localhost:${WEB_PORT}`);
});
