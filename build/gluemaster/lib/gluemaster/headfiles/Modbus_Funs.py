from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException, ConnectionException
import time
#uint16
CONTROL_MODE_ADR=0x02*256+0


DI1_EN_ADR=0x03*256+3#0ms滤波
DI1_FUN_ADR=DI1_EN_ADR-1
DI2_EN_ADR=DI1_EN_ADR+2#0ms滤波
DI2_FUN_ADR=DI2_EN_ADR-1

DI8_EN_ADR=0x03*256+17

DI9_EN_ADR=0x03*256+19

DO1_FUN_ADR=0x04*256+0

DI_MONITOR_ADR=0x0B*256+3
DO_MONITOR_ADR=DI_MONITOR_ADR+2
SPEED_COMMAND_SOURCE_ADR=0x06*256+2
POSITION_COMMAND_SOURCE_ADR=0x05*256+0#0脉冲0内部多段位2


OP_RETURN_EN_ADR=0x05*256+30
OP_RETURN_MODE_ADR=OP_RETURN_EN_ADR+1

EEPROM_SAVE_ADR=0x0C*256+13

#int32
MACHINE_OP_OFFSET_ADR=0x05*256+36

ABSOLUTE_POSITION_ADR=0x0B*256+7

MOTION1_ADR=0x11*256+12

#in16
SPEED_SET_ADR=0x06*256+3
SPEEDUP_TIME_ADR=SPEED_SET_ADR+2
SPEEDDOWN_TIME_ADR=SPEEDUP_TIME_ADR+1

MOTION1_MAXSPEED_ADR=MOTION1_ADR+2



# MOTION2_ADR=MOTION1_ADR+5
# MOTION3_ADR=MOTION2_ADR+5
# MOTION4_ADR=MOTION3_ADR+5


MULTISPEED_ADR=[0x12*256+20,0x12*256+23,0x12*256+26,0x12*256+29]
MULTISPEED_TIME_ADR=[0x12*256+20+1,0x12*256+23+1,0x12*256+26+1,0x12*256+29+1]
MULTISPEED_MODBUSCHOOSE_ADR=0x12*256+1
MULTISPEED_MODBUSMODECHOOSE_ADR=0x12*256#0单次1循环(01段数由h1201设置)2DI切换
# #创建Modbus客户端
# client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=57600) 

def read16(client,slave, address,maxretry=5):
    retry=0
    while retry<maxretry:
        try:
            #读取寄存器
            response = client.read_holding_registers(address=address, count=1, slave=slave)
            if response.isError():
                print("Failed to read registers")
                return None
            # print("Values: ", response.registers)
            return response.registers[0]
        except ModbusException as e:
            if retry==maxretry-1:
                print("Modbus异常:", e)
            time.sleep(0.04)
        except Exception as e:
            # 捕获除ModbusException之外的所有异常
            print(f"An error occurred: {e}")
            return None
        retry=retry+1

def combine_to_int32(num):
    high_uint16 = num[1]
    low_uint16 = num[0]
    uint32 = (high_uint16 << 16) | low_uint16
    return uint32 if uint32 < 0x80000000 else uint32 - 0x100000000

def read32(client,slave, address,maxretry=5):
    retry=0
    while retry<maxretry:
        try:
            #读取寄存器
            response = client.read_holding_registers(address=address, count=2, slave=slave)
            if response.isError():
                print("Failed to read registers")
                return None
            response = combine_to_int32(response.registers)
            # print("Values: ", response)
            return response
        except ModbusException as e:
            if retry==maxretry-1:
                print("Modbus异常:", e)
            time.sleep(0.04)
        except Exception as e:
            # 捕获除ModbusException之外的所有异常
            print(f"An error occurred: {e}")
            return None
        retry=retry+1
    
def write16(client,slave, address, value,maxretry=5):
    retry=0
    while retry<maxretry:
        try:
            if value<0:
                value=value&0xFFFF
            #写入单个寄存器
            write_response = client.write_register(address=address, value=value, slave=slave)
            if write_response.isError():
                print("Failed to write register")
            return None
        except ModbusException as e:
            if retry==maxretry-1:
                print("Modbus异常:", e)
            time.sleep(0.04)
        except Exception as e:
            # 捕获除ModbusException之外的所有异常
            print(f"An error occurred: {e}")
            return None
        retry=retry+1
    

def split_int32_to_uint16_list(value):
    # 将32位有符号整数转为无符号表示
    uint32 = value if value >= 0 else (1 << 32) + value
    # 拆分高低16位
    high = (uint32 >> 16) & 0xFFFF
    low = uint32 & 0xFFFF
    return [low, high]

def write32(client,slave, address, values,maxretry=5):
    retry=0
    while retry<maxretry:
        try:
            #写入多个寄存器
            write_response = client.write_registers(address=address, values=split_int32_to_uint16_list(values), slave=slave)
            if write_response.isError():
                print("Failed to write multiple registers")
            # print("Multiple registers written successfully")
            return None
        except ModbusException as e:
            if retry==maxretry-1:
                print("Modbus异常:", e)
            time.sleep(0.04)
        except Exception as e:
            # 捕获除ModbusException之外的所有异常
            print(f"An error occurred: {e}")
        retry=retry+1

# if __name__ == "__main__":
#     try:
#         if client.connect(): 
#             print("Modbus RTU Client Connected")
#             # write32(1, 17*256+12, -2000)
#             # read32(1, 17*256+12 )
#             write16(1, 3*256+3, 0)
#             write16(1, 3*256+5, 0)

#             # write16(1, 3*256+3, 1)
#             # write16(1, 3*256+5, 1)
#             # client.close()  # 关闭连接
#             print("Modbus RTU Client Close")
#         else:
#             print("无法连接到Modbus设备")
#     except ConnectionException as e:
#             print("连接异常:", e)