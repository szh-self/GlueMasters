import gluemaster.headfiles.Modbus_Funs as MF
import gluemaster.headfiles.gluemaster_config as gm_config
# import Modbus_Funs as MF
# import gluemaster_config as gm_config

import time
import Hobot.GPIO as GPIO

MACHINE_EN_ADR=MF.DI8_EN_ADR
MULTIPOS_EN_ADR=MF.DI9_EN_ADR
SPEED_SET_ADR=MF.SPEED_SET_ADR
POSITION_COMMAND_SOURCE_ADR=MF.POSITION_COMMAND_SOURCE_ADR
ONEPULSELENGTH=0.01

def Speed_Real2RPM(realspeed):#realspeed= mm/s 转换成RPM时会四舍五入取整
    return round(realspeed*6)

def Speed_RPM2Real(rpmspeed):
    return rpmspeed/6

# def PositionModeSet(client,slave,eepflag):#设置为位置控制,eepflag=1永久写入
#     MF.write16(client,slave,MF.CONTROL_MODE_ADR,1)
#     MF.write16(client,slave,MF.DO1_FUN_ADR,5)
#     if eepflag:
#         MF.write16(client,slave,MF.EEPROM_SAVE_ADR,1)



# def ShutDown(client,slave):
#     MF.write16(client,slave,MACHINE_EN_ADR,0)
#     MF.write16(client,slave,MULTIPOS_EN_ADR,0)

# def Start(client,slave):
#     MF.write16(client,slave,MACHINE_EN_ADR,1)



# def OPreturn(client,slave,flag):#flag=1强制复位(1sdelay)，flag=0此函数只第一次调用有效,会永久保存为位置模式
#     ShutDown(client,slave)
#     time.sleep(1)
#     PositionModeSet(client,slave,eepflag=0)
#     time.sleep(1)
#     if flag:
#         MF.write16(client,slave,MACHINE_EN_ADR,1)
#         time.sleep(1)
#         MF.write16(client,slave,MF.OP_RETURN_EN_ADR,4)
#         time.sleep(1)
#         for i in range(6000):
#             time.sleep(0.01)
#             if MF.read16(client,slave,MF.DO_MONITOR_ADR)&0x04:
#                 MF.write16(client,slave,MACHINE_EN_ADR,0)
#                 print("op set")
#                 break
#             elif i==5999:
#                 print("Error:time out!!!")    
#     else:
#         MF.write16(client,slave,MACHINE_EN_ADR,1)
#         for i in range(6000):
#             time.sleep(0.01)
#             if MF.read16(client,slave,MF.DO_MONITOR_ADR)&0x04:
#                 MF.write16(client,slave,MACHINE_EN_ADR,0)
#                 print("ok2")
#                 break
#             elif i==5999:
#                 print("Error:time out!!!") 
#     time.sleep(1)

class Driver:
    name='noname'
    def GetPos(self):
        return MF.read32(self.client,self.slave,MF.ABSOLUTE_POSITION_ADR)
    
    def PositionModeSet(self,pulse):#设置为位置控制
        self.ShutDown()
        time.sleep(0.1)
        if pulse&self.PulseMode:
            MF.write16(self.client,self.slave,POSITION_COMMAND_SOURCE_ADR,0)
        else:
            MF.write16(self.client,self.slave,POSITION_COMMAND_SOURCE_ADR,2)
        MF.write16(self.client,self.slave,MF.DO1_FUN_ADR,5)
        MF.write16(self.client,self.slave,MF.CONTROL_MODE_ADR,1)
        self.Start()
    
    def SpeedModeSet(self):#设置为速度控制
        self.ShutDown()
        time.sleep(0.1)
        MF.write16(self.client,self.slave,MF.CONTROL_MODE_ADR,0)
        MF.write16(self.client,self.slave,MF.SPEED_COMMAND_SOURCE_ADR,0)
        self.Start()
    
    
    def SpeedUp(self,v):#用于z轴速度模式下设置速度，v mm/s
        MF.write16(self.client,self.slave,MF.SPEED_SET_ADR,Speed_Real2RPM(v))
    
    def DO_Check(self,tip,checkcode,tsflag=0,waitflag=1,ticks=6000,errormessage="Error:time out!!!"):#检查是否定位完成
        if waitflag:
            time.sleep(0.4)
            for i in range(ticks):
                time.sleep(0.01)
                if bool(MF.read16(self.client,self.slave,MF.DO_MONITOR_ADR)&checkcode)^tsflag:
                    print(self.name,tip)
                    return True
                elif i==ticks-1:
                    print(errormessage)  
                    return False
        else:
            if bool(MF.read16(self.client,self.slave,MF.DO_MONITOR_ADR)&checkcode)^tsflag:
                print(self.name,tip)
                return True
            else:
                return False
    
    def DI_Check(self,tip,checkcode,tsflag=0,waitflag=1,ticks=6000,errormessage="Error:time out!!!"):#检查原点开关光电输入
        if waitflag:
            time.sleep(0.4)
            for i in range(ticks):
                time.sleep(0.01)
                if bool(MF.read16(self.client,self.slave,MF.DI_MONITOR_ADR)&checkcode)^tsflag:
                    print(self.name,tip)
                    return True
                elif i==ticks-1:
                    print(errormessage)  
                    return False
        else:
            if bool(MF.read16(self.client,self.slave,MF.DI_MONITOR_ADR)&checkcode)^tsflag:
                print(self.name,tip)
                return True
            else:
                return False
    
    
    def ShutDown(self):
        MF.write16(self.client,self.slave,MACHINE_EN_ADR,0)
        MF.write16(self.client,self.slave,MULTIPOS_EN_ADR,0)
        
    def OPreturn(self,flag):#flag=1强制复位(1sdelay)，flag=0此函数只第一次调用有效,会变为位置模式
        self.PositionModeSet(pulse=1)
        time.sleep(0.1)
        if flag:
            time.sleep(0.1)
            MF.write16(self.client,self.slave,MF.OP_RETURN_EN_ADR,4)
            
    def Initialize(self):#寻找原点开关，作为Recrough_Pos
        match self.InitializeStep:
            case 0:
                self.OPreturn(flag=1)
                time.sleep(0.2)
                self.InitializeStep=self.InitializeStep+1
            case 1:
                if self.DO_Check(tip='op set',checkcode=gm_config.CHECK_HOMEATTAIN,waitflag=0):
                    self.InitializeStep=self.InitializeStep+1
                    print(self.name,' op Pos ',self.GetPos())
            case 2:
                if self.PulseMode:
                    self.PulseSpeedUp(-gm_config.HoFindV_H)
                else:
                    self.SpeedModeSet()
                    self.SpeedUp(-gm_config.HoFindV_H)
                self.InitializeStep=self.InitializeStep+1
                time.sleep(0.2)
            case 3:
                if self.DI_Check(tip='HomeSwitch stop',checkcode=gm_config.CHECK_HOMESWITCH,waitflag=0):
                    if self.PulseMode:
                        self.PulseSpeedUp(0)
                    else:
                        self.SpeedUp(gm_config.HoFindV_L)
                    self.InitializeStep=self.InitializeStep+1
            case 4:
                if self.DO_Check(tip='HomeSwitch back',checkcode=gm_config.CHECK_COIN,waitflag=0):
                    if self.PulseMode:
                        self.PulseSpeedUp(gm_config.HoFindV_L)
                    else:
                        self.SpeedUp(gm_config.HoFindV_L)
                    self.InitializeStep=self.InitializeStep+1
            case 5:
                if self.DI_Check(tip='HomeSwitch leave',checkcode=gm_config.CHECK_HOMESWITCH,waitflag=0,tsflag=1):
                    if self.PulseMode:
                        self.PulseSpeedUp(0)
                    else:
                        self.SpeedUp(0)
                    self.InitializeStep=self.InitializeStep+1
            case 6:
                if self.DO_Check(tip='HomeSwitch Record',checkcode=gm_config.CHECK_COIN,waitflag=0):
                    if not self.PulseMode:
                        self.PositionModeSet(pulse=0)
                    self.HomeSwitchPos=self.GetPos()
                    self.InitializeStep=self.InitializeStep+1
                    return True
            case 7:
                return True
        return False

    def MoveToPosition(self,position,realspeed,wait=1):
        
        MF.write32(self.client,self.slave,MF.MOTION1_ADR,position)
        MF.write16(self.client,self.slave,MF.MOTION1_MAXSPEED_ADR,abs(Speed_Real2RPM(realspeed)))
        MF.write16(self.client,self.slave,MACHINE_EN_ADR,1)
        MF.write16(self.client,self.slave,MULTIPOS_EN_ADR,1)
        if wait:
            time.sleep(0.2)
            for i in range(1000):
                time.sleep(0.01)
                if MF.read16(self.client,self.slave,MF.DO_MONITOR_ADR)&0x01:
                    MF.write16(self.client,self.slave,MULTIPOS_EN_ADR,0)
                    break
                elif i==9999:
                    print(self.name+"Error:time out!!!")    
    
    # def PulseMoveLength(self,v,t,beltspeed):#v+- beltspeed+-
        
    #     f1=round(abs((v+beltspeed))/ONEPULSELENGTH)
    #     f2=round(abs(beltspeed)/ONEPULSELENGTH)
    #     if f1>0:
    #         GPIO.output(self.pin_d,GPIO.HIGH)
    #     else:
    #         GPIO.output(self.pin_d,GPIO.LOW)
    #     if f1:
    #         self.pul.ChangeFrequency(f1)
    #     else:
    #         self.pul.stop()
    #     self.pul.start(50)
    #     time.sleep(t-0.0014)
    #     if f2==0:
    #         self.pul.stop()
    #     else:
    #         self.pul.ChangeFrequency(f2)
        
    def PulseSpeedUp(self,v):
        f=round(abs(v)/ONEPULSELENGTH)
        if v>0:
            GPIO.output(self.pin_d,GPIO.HIGH)
        else:
            GPIO.output(self.pin_d,GPIO.LOW)
        if f:
            self.pul.ChangeFrequency(f)
            self.pul.start(50)
        else:
            self.pul.stop()
    
    def Start(self):
        MF.write16(self.client,self.slave,MACHINE_EN_ADR,1)
    
    def __init__(self,client,slave,name,pulsemode,pin_p=0,pin_d=0):
        GPIO.setmode(GPIO.BOARD)
        self.client=client
        self.slave=slave
        self.name=name
        self.PulseMode=pulsemode
        self.HomeSwitchPos=0
        if pin_p:
            self.pin_p=pin_p
            self.pul=GPIO.PWM(self.pin_p,10000)
            self.pul.ChangeDutyCycle(50)
        if pin_d:
            self.pin_d=pin_d
            GPIO.setup(self.pin_d, GPIO.OUT,initial=GPIO.LOW)
            
        self.InitializeStep=0
        
    def __del__(self):
        if hasattr(self,"pul"):
            self.pul.stop()
        if hasattr(self,"pin_d")&hasattr(self,"pin_p"):
            print("clear",self.pin_d," ",self.pin_p)
            GPIO.cleanup([self.pin_d,self.pin_p])
        self.OPreturn(flag=1)
        self.ShutDown()
        













