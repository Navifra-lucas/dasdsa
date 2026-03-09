import asyncio
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusSlaveContext, ModbusServerContext
from pymodbus.server import ModbusSerialServer
import logging

# 로그 설정
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# 데이터 블록을 설정하여 고정된 이산 입력 값 준비
fixed_inputs = [True, False, True, True, False]
discrete_inputs = ModbusSequentialDataBlock(0, [int(val) for val in fixed_inputs])

# RTU에서 사용할 Modbus Slave Context 설정
store = ModbusSlaveContext(di=discrete_inputs, zero_mode=True)
context = ModbusServerContext(slaves=store, single=True)

async def run_modbus_server():
    print("Starting Modbus RTU Server...")
    server = ModbusSerialServer(context, port="/dev/ttyV0", baudrate=9600, parity="N", stopbits=1, bytesize=8, method="rtu")
    await server.serve_forever()

# 메인 이벤트 루프 실행
try:
    asyncio.run(run_modbus_server())
except KeyboardInterrupt:
    print("Stopping Modbus server...")
