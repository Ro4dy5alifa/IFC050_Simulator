require("dotenv").config();
const path = require("path");
const express = require("express");
const ModbusRTU = require("modbus-serial");

const MODBUS_PORT = Number(process.env.MODBUS_PORT || 502);
const MODBUS_HOST = process.env.MODBUS_HOST || "0.0.0.0";
const WEB_PORT = Number(process.env.WEB_PORT || 8080);
const METER_COUNT = 3;
const TICK_MS = 100;

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
    holding,
    targetSpeed: 0.0,
    filtSpeed: 0.0,
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
    statusSensor: 0,
    statusDevice: 0,
    wordOrder: "ABCD",
    logs: [],
    pending: null
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

function flushPending(m) {
  if (!m.pending) return;
  const p = m.pending;
  const range = p.end > p.start ? `${p.start}-${p.end}` : `${p.start}`;
  const qty = p.end - p.start + 1;
  m.logs.push(`[${p.time}] ${p.fc} ${p.label} ${range} (${qty} reg${qty > 1 ? "s" : ""})`);
  if (m.logs.length > MAX_LOGS) m.logs.splice(0, m.logs.length - MAX_LOGS);
  m.pending = null;
}

function logRead(m, fc, label, addr) {
  if (m.pending && m.pending.fc === fc && addr === m.pending.end + 1) {
    m.pending.end = addr;
    clearTimeout(m.pending.timer);
    m.pending.timer = setTimeout(() => flushPending(m), 150);
    return;
  }
  flushPending(m);
  m.pending = { fc, label, start: addr, end: addr, time: ts(), timer: setTimeout(() => flushPending(m), 150) };
}

function logWrite(m, fc, label, addr, value) {
  flushPending(m);
  m.logs.push(`[${ts()}] ${fc} ${label} ${addr} <- ${value}`);
  if (m.logs.length > MAX_LOGS) m.logs.splice(0, m.logs.length - MAX_LOGS);
}

const meters = [];
for (let i = 1; i <= METER_COUNT; i++) meters.push(makeMeter(i));

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

  m.statusSensor = m.emptyPipe ? 0x00000001 : 0;
  m.statusDevice = m.error ? 0x00000001 : 0;
}

let lastTick = Date.now();
setInterval(() => {
  const now = Date.now();
  const dt = (now - lastTick) / 1000;
  lastTick = now;
  for (const m of meters) {
    const tau = Math.max(readFloatBE(m.holding, 3004), 0.05);
    m.filtSpeed += (m.targetSpeed - m.filtSpeed) * Math.min(dt / tau, 1);
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

function meterOrNull(unitID) {
  return meters[unitID - 1] || null;
}

const vector = {
  getInputRegister: (addr, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC04 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve(0);
      }
      const n = normInput(addr);
      const block = inputRegisterBlock(m);
      if (n < 0 || n >= block.length) {
        logRead(m, "FC04", "InputReg", addr);
        return resolve(0);
      }
      logRead(m, "FC04", "InputReg", addr);
      resolve(block[n]);
    }),
  getDiscreteInput: (addr, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC02 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve(false);
      }
      logRead(m, "FC02", "DiscreteIn", addr);
      resolve(getCoilState(m, addr));
    }),
  getHoldingRegister: (addr, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC03 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve(0);
      }
      const n = normHolding(addr);
      logRead(m, "FC03", "HoldingReg", addr);
      resolve(n >= 0 && n < m.holding.length ? m.holding[n] : 0);
    }),
  getCoil: (addr, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC01 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve(false);
      }
      logRead(m, "FC01", "Coil", addr);
      resolve(getCoilState(m, addr));
    }),
  setCoil: (addr, value, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC05 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve();
      }
      setCoilState(m, addr, value);
      logWrite(m, "FC05", "Coil", addr, value ? 1 : 0);
      resolve();
    }),
  setRegister: (addr, value, unitID) =>
    new Promise((resolve) => {
      const m = meterOrNull(unitID);
      if (!m) {
        slog(`REJECTED FC06/16 unit=${unitID} addr=${addr}: unknown unit ID (simulator serves 1-${METER_COUNT})`);
        return resolve();
      }
      const n = normHolding(addr);
      if (n >= 0 && n < m.holding.length) m.holding[n] = value & 0xffff;
      logWrite(m, "FC06/16", "HoldingReg", addr, value);
      resolve();
    })
};

const serverTCP = new ModbusRTU.ServerTCP(vector, {
  host: MODBUS_HOST,
  port: MODBUS_PORT,
  debug: false
});

serverTCP.on("socketError", (err) => console.error("Modbus socket error:", err.message));
console.log(`IFC050 Modbus TCP server listening on ${MODBUS_HOST}:${MODBUS_PORT} (unit IDs 1-${METER_COUNT})`);

const app = express();
app.use(express.json());
app.use(express.static(path.join(__dirname, "public")));

function meterState(m) {
  return {
    id: m.id,
    targetSpeed: m.targetSpeed,
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
    wordOrder: m.wordOrder,
    logs: m.logs,
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

app.get("/api/state", (req, res) => {
  res.json({ meters: meters.map(meterState), serverLog });
});

app.post("/api/meter/:id", (req, res) => {
  const m = meters[Number(req.params.id) - 1];
  if (!m) return res.status(404).json({ error: "unknown meter" });
  const b = req.body || {};
  if (typeof b.flowSpeed === "number") m.targetSpeed = b.flowSpeed;
  if (typeof b.emptyPipe === "boolean") m.emptyPipe = b.emptyPipe;
  if (typeof b.error === "boolean") m.error = b.error;
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
