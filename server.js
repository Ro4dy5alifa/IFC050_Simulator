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
    statusDevice: 0
  };
}

const meters = [];
for (let i = 1; i <= METER_COUNT; i++) meters.push(makeMeter(i));

function meterByUnit(unitID) {
  const m = meters[unitID - 1];
  if (!m) throw new Error("Invalid unit ID " + unitID);
  return m;
}

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
  return r;
}

function getCoilState(m, addr) {
  switch (addr) {
    case 3000: return m.c1run;
    case 3001: return m.c2run;
    case 3002: return false;
    case 3003: return false;
    case 3004: return false;
    default: throw new Error("Illegal coil address " + addr);
  }
}

function setCoilState(m, addr, value) {
  switch (addr) {
    case 3000: m.c1run = !!value; break;
    case 3001: m.c2run = !!value; break;
    case 3003: if (value) m.counter1 = 0; break;
    case 3004: if (value) m.counter2 = 0; break;
    default: throw new Error("Illegal coil address " + addr);
  }
}

const vector = {
  getInputRegister: (addr, unitID) =>
    new Promise((resolve, reject) => {
      try {
        const m = meterByUnit(unitID);
        const n = normInput(addr);
        const block = inputRegisterBlock(m);
        if (n < 0 || n >= block.length) return reject(new Error("Illegal input register " + addr));
        resolve(block[n]);
      } catch (e) { reject(e); }
    }),
  getHoldingRegister: (addr, unitID) =>
    new Promise((resolve, reject) => {
      try {
        const m = meterByUnit(unitID);
        const n = normHolding(addr);
        if (n < 0 || n >= m.holding.length) return reject(new Error("Illegal holding register " + addr));
        resolve(m.holding[n]);
      } catch (e) { reject(e); }
    }),
  getCoil: (addr, unitID) =>
    new Promise((resolve, reject) => {
      try { resolve(getCoilState(meterByUnit(unitID), addr)); } catch (e) { reject(e); }
    }),
  setCoil: (addr, value, unitID) =>
    new Promise((resolve, reject) => {
      try {
        setCoilState(meterByUnit(unitID), addr, value);
        console.log(`[unit ${unitID}] coil ${addr} <- ${value ? 1 : 0}`);
        resolve();
      } catch (e) { reject(e); }
    }),
  setRegister: (addr, value, unitID) =>
    new Promise((resolve, reject) => {
      try {
        const m = meterByUnit(unitID);
        const n = normHolding(addr);
        if (n < 0 || n >= m.holding.length) return reject(new Error("Illegal holding register " + addr));
        m.holding[n] = value & 0xffff;
        console.log(`[unit ${unitID}] holding ${addr} <- ${value}`);
        resolve();
      } catch (e) { reject(e); }
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
  res.json({ meters: meters.map(meterState) });
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
