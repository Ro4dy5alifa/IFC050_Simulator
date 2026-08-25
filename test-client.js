const ModbusRTU = require("modbus-serial");

function f32(buf) { return buf.readFloatBE(0); }
function f64(buf) { return buf.readDoubleBE(0); }

(async () => {
  const client = new ModbusRTU();
  await client.connectTCP("127.0.0.1", { port: 502 });
  client.setTimeout(3000);

  for (const id of [1, 2, 3]) {
    client.setID(id);

    // set flow 2.5 m/s via web API equivalent: here via holding reg direction read
    const ir = await client.readInputRegisters(30000, 20);
    console.log(`unit ${id} FC04 @30000: speed=${f32(ir.buffer.slice(0,4)).toFixed(3)} vol=${f32(ir.buffer.slice(4,8)).toFixed(5)} mass=${f32(ir.buffer.slice(8,12)).toFixed(3)} optime=${f32(ir.buffer.slice(12,16)).toFixed(1)} c1=${f64(ir.buffer.slice(16,24)).toFixed(3)} c2=${f64(ir.buffer.slice(24,32)).toFixed(3)}`);

    // offset-based addressing
    const ir2 = await client.readInputRegisters(0, 4);
    console.log(`unit ${id} FC04 @0: speed=${f32(ir2.buffer.slice(0,4)).toFixed(3)}`);

    const hr = await client.readHoldingRegisters(43004, 2);
    console.log(`unit ${id} FC03 @43004 time constant=${f32(hr.buffer).toFixed(2)}`);

    const coils = await client.readCoils(3000, 5);
    console.log(`unit ${id} FC01 coils 3000-3004: ${coils.data.join(",")}`);
  }

  // FC05: start counter 1 on unit 1, FC16: write time constant
  client.setID(1);
  await client.writeCoil(3000, true);
  await client.writeRegisters(43004, [0x40a0, 0x0000]); // 5.0 s
  const hr = await client.readHoldingRegisters(43004, 2);
  console.log(`unit 1 after FC16: time constant=${f32(hr.buffer).toFixed(2)}`);
  const coils = await client.readCoils(3000, 1);
  console.log(`unit 1 after FC05: coil 3000=${coils.data[0]}`);

  client.close(() => process.exit(0));
})().catch(e => { console.error("FAIL:", e.message); process.exit(1); });
