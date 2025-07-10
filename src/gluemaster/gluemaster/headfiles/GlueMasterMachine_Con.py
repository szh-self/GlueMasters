#!/usr/bin/env python3

import gluemaster.headfiles.Driver_Con_FunsPULSE as DC
import gluemaster.headfiles.gluemaster_config as gm_config
# import Driver_Con_FunsPULSE as DC
# import gluemaster_config as gm_config

import time
import math
import csv
from pynput import keyboard
import numpy as np
# from threading import Thread

class GlueMasterMachine:
    def __init__(self,modbusport,modbusbaudrate,beltspeed=0):
        self.client = DC.MF.ModbusSerialClient(port=modbusport, baudrate=modbusbaudrate)
        self.driverz=DC.Driver(self.client,3,"z",0)
        self.driverx=DC.Driver(self.client,1,"x",1,pin_p=33,pin_d=16)
        self.drivery=DC.Driver(self.client,2,"y",1,pin_p=32,pin_d=36)
        self.Pos=[-1000,-1000,-200]#p
        self.Initialize()
        self.beltspeed=beltspeed
        
        self.HomeSwitchPos=[self.driverx.HomeSwitchPos,self.drivery.HomeSwitchPos,self.driverz.HomeSwitchPos]#等待粗识别时胶头离传送带的高度(mm)
        
        self.ReadCalibrationData()
    
    def Initialize(self,ticks=6000):
        self.driverz.InitializeStep=0
        self.driverx.InitializeStep=0
        self.drivery.InitializeStep=0
        for i in range(ticks):
            time.sleep(0.01)
            resultz=self.driverz.Initialize()
            resultx=self.driverx.Initialize()
            resulty=self.drivery.Initialize()
            if resultz and resultx and resulty:
                print('HomeSwitchPos:----')
                self.UpdatePosNow()
                print('HomeSwitchPos:----')
                return None
            elif i==ticks-1:
                print('Ini timeout')

    def GetCSVMethod(self,path):
        try:
            self.CurrentCSVMethod = []
            with open(path, 'r',encoding='utf-8') as csvfile:
                csv_reader = csv.reader(csvfile)
                for row in csv_reader:
                    temprow=[]
                    temprow.append(row[0])
                    for i in range(1,5):
                        temprow.append(eval(row[i]))
                    
                    self.CurrentCSVMethod.append(temprow)
                    print(temprow)
        except FileNotFoundError:
            print("File is not found.")
        except PermissionError:
            print("You don't have permission to access this file.")
    
    def ReadCalibrationData(self):
        try:
            with open(gm_config.gluemasterfilepath+'/CalibrationData.csv', 'r') as csvfile:
            # with open('/home/sunrise/文档/GM_ws/src/gluemaster/gluemaster/headfiles/CalibrationData.csv', 'r') as csvfile:
                csv_reader = list(csv.reader(csvfile))
                if len(csv_reader)>0:
                    for row in csv_reader:
                        for i in range(3):
                            row[i]=eval(row[i])
                        print('read calibration data:')
                        self.Rec_roughPos=row[0]
                        print('Rec_roughPos: ',self.Rec_roughPos)
                        self.Rec_rough2accuratePos=row[1]
                        print('Rec_rough2accuratePos: ',self.Rec_rough2accuratePos)
                        self.Rec_accurate2realPos=row[2]
                        print('Rec_accurate2realPos: ',self.Rec_accurate2realPos)
                        self.Z0=self.Rec_roughPos[2]+self.Rec_rough2accuratePos[2]+self.Rec_accurate2realPos[2]
                        print('Z0: ',self.Z0)
                        #出胶头碰到传送带时的z坐标
                        self.Rec_roughHeight=self.Pos2Height(self.Rec_roughPos[2])
                        print('Rec_roughHeight: ',self.Rec_roughHeight)
                        self.Rec_accurateHeight=self.Rec_roughHeight+self.Rec_rough2accuratePos[2]*0.01
                        print('Rec_accurateHeight: ',self.Rec_accurateHeight)
                else:
                    print('read error:Bad CalibrationData')
        except FileNotFoundError:
            print("File is not found.")
        except PermissionError:
            print("You don't have permission to access this file.")
    
    def CatchUpBelt(self):
        self.drivery.PulseSpeedUp(self.beltspeed)
        # time.sleep(0.05)
    
    def Start(self):
        self.driverz.Start()
        self.driverx.Start()
        self.drivery.Start()
    
    def UpdatePosNow(self):
        self.Pos[0]=self.driverx.GetPos()        
        self.Pos[1]=self.drivery.GetPos()
        self.Pos[2]=self.driverz.GetPos()
        print(self.Pos)
        
    def Height2Pos(self,Height):#将传入的距离传送带的高度转换为z坐标   
        return round(self.Z0+Height*100)

    def Pos2Height(self,Pos_p):#将传入的距离传送带的高度转换为z坐标   
        return (Pos_p-self.Z0)/100

    def Draw(self):#出胶
        pass
    
    def StopDraw(self):#停止出胶
        pass
    
    def SlowReturn(self,Vy,Posylimit_p,Posx_p,Posz_p):#Posx为x轴坐标（p）Vy(mm/s)用于等待识别，y缓慢向-1000前进到达后停下，x回到识别位置
        self.UpdatePosNow()
        if self.Pos[1]<Posylimit_p:
            self.Move2Pos(Pos_mm=[(Posx_p-self.Pos[0])*0.01,0,self.Pos2Height(Posz_p)],t=0.5,Draw=0)
            self.drivery.PulseSpeedUp(Vy)
        else:
            self.Move2Pos(Pos_mm=[(Posx_p-self.Pos[0])*0.01,0,self.Pos2Height(Posz_p)],t=0.5,Draw=0)
            self.drivery.PulseSpeedUp(0)
        
    def SafetyCheck(self):
        self.UpdatePosNow()
        if self.Pos[1]<self.HomeSwitchPos[1]:
            return False
        return True
        
        
    def Move2Pos(self,Pos_mm,t,Draw=0):#Version:time.sleep最高封装，最低精度
        # self.UpdatePosNow()
        #Pos[0-1]为相对于当前位置的坐标(向量0x1y mm) Draw=1表示要出胶 Height(Pos[2])是距离传送带的高度
        self.driverz.MoveToPosition(self.Height2Pos(Pos_mm[2]),100,wait=1)
        t=max(t,0.0017)
        Vx=Pos_mm[0]/t
        Vy=Pos_mm[1]/t-self.beltspeed
        self.driverx.PulseSpeedUp(Vx)
        self.drivery.PulseSpeedUp(Vy)
        if Draw:
            self.Draw()
        time.sleep(t-0.0017)
        self.driverx.PulseSpeedUp(0)
        self.drivery.PulseSpeedUp(-self.beltspeed)
        if Draw:
            self.StopDraw()
        # self.UpdatePosNow()
    def Move2Pos_Absolute(self,Pos_p,wait=1,tip='Move to absolute pos complete'):
        #Pos 单位p
        self.driverz.MoveToPosition(position=Pos_p[2],realspeed=1.5*gm_config.HoFindV_H,wait=1)
        self.driverx.PositionModeSet(pulse=0)
        self.drivery.PositionModeSet(pulse=0)
        self.driverx.MoveToPosition(position=Pos_p[0],realspeed=1.5*gm_config.HoFindV_H,wait=0)
        self.drivery.MoveToPosition(position=Pos_p[1],realspeed=1.5*gm_config.HoFindV_H,wait=0)
        time.sleep(0.3)
        if wait:
            self.driverz.DO_Check(tip=tip+' '+str(Pos_p[2]),checkcode=gm_config.CHECK_COIN)
            self.driverx.DO_Check(tip=tip+' '+str(Pos_p[0]),checkcode=gm_config.CHECK_COIN)
            self.drivery.DO_Check(tip=tip+' '+str(Pos_p[1]),checkcode=gm_config.CHECK_COIN)
        self.driverx.PositionModeSet(pulse=1)
        self.drivery.PositionModeSet(pulse=1)
    
    def GetAbsolutePos_p(self,RelativePos_p):
        self.UpdatePosNow()
        tempPos=[]
        for i in range(len(RelativePos_p)):
            tempPos.append(RelativePos_p[i]+self.Pos[i])
        return tuple(tempPos)
    
    def Move2Pos_Relative(self,Pos_p,wait=1,tip='Move to relative pos complete'):
        AbsolutePos_p=self.GetAbsolutePos_p(Pos_p)
        self.Move2Pos_Absolute(AbsolutePos_p,wait=wait,tip=tip)
            
            
    
    # def DrawCircle(self,CenterPos,t,height,Draw=1):#以当前位置为起点画圆
    #     #CenterPos为圆心相对于圆起点位置的坐标(向量0x1y mm)
    #     slicenum=1000
    #     mininterval=0.0021
    #     phi=math.atan(CenterPos[1]/CenterPos[0])#计算初相
    #     # i0=round((slicenum*phi)/(2*math.pi))
    #     C=2*math.pi*math.sqrt(CenterPos[0]**2+CenterPos[1]**2)
    #     V=C/t
    #     interval=max(t/slicenum,mininterval)
    #     print('circle inetrval',interval)
    #     self.driverz.MoveToPosition(self.Height2Pos(height),100,wait=1)
    #     if Draw:
    #         self.Draw()
    #     for i in range(1,slicenum+1):
    #         rad=2*math.pi*i/slicenum
    #         self.driverx.PulseSpeedUp(V*math.sin(rad+phi))
    #         self.drivery.PulseSpeedUp(-V*math.cos(rad+phi)-self.beltspeed)
    #         time.sleep(interval-mininterval)
    #     self.driverx.PulseSpeedUp(0)
    #     self.drivery.PulseSpeedUp(-self.beltspeed)
    #     if Draw:
    #         self.StopDraw()

    def DrawCircle(self,CenterPos,t,height,Draw=1,slicenum=36):#以当前位置为起点画圆CenterPOs:nparray
        #CenterPos为圆心相对于圆起点位置的坐标(向量0x1y mm)
        anglestep=360/slicenum
        mininterval=0.0017
        R=math.sqrt(CenterPos[0]**2+CenterPos[1]**2)
        theta=math.pi/slicenum
        length=2*R*math.sin(theta)
        phi=np.arctan2(CenterPos[1],CenterPos[0])+theta#计算初相
        
        Motion=np.array([length*math.sin(phi),-length*math.cos(phi)])
        interval=max(t/slicenum,mininterval)
        print('circle inetrval',interval)
        self.driverz.MoveToPosition(self.Height2Pos(height),100,wait=1)
        if Draw:
            self.Draw()
        for i in range(slicenum):
            Vx=Motion[0]/interval
            Vy=Motion[1]/interval-self.beltspeed
            self.driverx.PulseSpeedUp(Vx)
            self.drivery.PulseSpeedUp(Vy)
            time.sleep(interval-mininterval)
            Motion=gm_config.rotate_vector(Motion,anglestep)
        self.driverx.PulseSpeedUp(0)
        self.drivery.PulseSpeedUp(-self.beltspeed)
        if Draw:
            self.StopDraw()

    def ExcecuteMethodWithAngle(self,angle):#angle +:顺时针
        for step in self.CurrentCSVMethod:
            Vector=np.array(step[1])
            Vector=gm_config.VectorAxisCam2Real(Vector=Vector,flag=1)
            RotatedVector=gm_config.rotate_vector(Vector,angle)
            print('CSVangle',angle,'reslut:',Vector)
            # RotatedList=RotatedVector.tolist()
            match step[0]:
                case 'movement':
                    self.Move2Pos([RotatedVector[0],RotatedVector[1],step[3]],step[2],Draw=step[4])
                case 'line':
                    self.Move2Pos([RotatedVector[0],RotatedVector[1],step[3]],step[2],Draw=step[4])
                case 'point':
                    self.Move2Pos([0,0,step[3]],step[2],Draw=step[4])
                case 'circle':
                    self.DrawCircle(RotatedVector,step[2],step[3],Draw=step[4])

    def ShutDown(self):
        self.driverx.ShutDown()
        self.drivery.ShutDown()
        self.driverz.ShutDown()
    
    def Reset(self,wait=0):
        self.driverx.SpeedUp(0)
        self.drivery.SpeedUp(0)
        self.driverx.OPreturn(flag=1)
        self.drivery.OPreturn(flag=1)
        self.driverz.OPreturn(flag=1)
        if wait:
            self.driverx.DO_Check(tip=" op set",checkcode=gm_config.CHECK_COIN)
            self.drivery.DO_Check(tip=" op set",checkcode=gm_config.CHECK_COIN)
            self.driverz.DO_Check(tip=" op set",checkcode=gm_config.CHECK_COIN)
    
    
    def Key_response(self,v,CalibrationV,key,current_keys):
        match key:
            case keyboard.KeyCode(char='w'):#y轴负向前进
                self.drivery.PulseSpeedUp(-v)
            case keyboard.KeyCode(char='a'):#x+
                self.driverx.PulseSpeedUp(v)
            case keyboard.KeyCode(char='s'):#y轴正向前进
                self.drivery.PulseSpeedUp(v)
            case keyboard.KeyCode(char='d'):#x-
                self.driverx.PulseSpeedUp(-v)
            case keyboard.Key.shift_l:#z-
                self.driverz.SpeedUp(-v)
            case keyboard.Key.space:#z+
                self.driverz.SpeedUp(v)
            case keyboard.KeyCode(char='r'):#按下时10mm/s不按下时1mm/s
                print('speed change')
                for k in current_keys:
                    match k:
                        case keyboard.KeyCode(char='w'):#y轴负向前进
                            self.drivery.PulseSpeedUp(-CalibrationV)
                        case keyboard.KeyCode(char='a'):#x+
                            self.driverx.PulseSpeedUp(CalibrationV)
                        case keyboard.KeyCode(char='s'):#y轴正向前进
                            self.drivery.PulseSpeedUp(CalibrationV)
                        case keyboard.KeyCode(char='d'):#x-
                            self.driverx.PulseSpeedUp(-CalibrationV)
                        case keyboard.Key.shift_l:#z-
                            print('speed change2')
                            self.driverz.SpeedUp(-CalibrationV)
                        case keyboard.Key.space:#z+
                            self.driverz.SpeedUp(CalibrationV)    
                        case _:
                            pass
            case _:
                pass
    
    
    def __del__(self):
        del self.driverz
        del self.driverx
        del self.drivery
    # def Move2Pos(self,Pos,t,Draw=0):#Version:time.sleep最高精度，最低封装度
    #     #Pos[0-1]为相对于当前位置的坐标(向量0x1y mm) Draw=1表示要出胶 Height(Pos[2])是距离传送带的高度
    #     self.driverz.MoveToPosition(self.Height2Pos(Pos[2]),100,wait=1)
    #     Vx=Pos[0]/t
    #     Vy=Pos[1]/t+self.beltspeed
    #     f1x=round(abs(Vx)/DC.ONEPULSELENGTH)
    #     f1y=round(abs((Vy+self.beltspeed))/DC.ONEPULSELENGTH)
    #     f2y=round(abs(self.beltspeed)/DC.ONEPULSELENGTH)
    #     if Vx>0:
    #         DC.GPIO.output(self.driverx.pin_d,DC.GPIO.HIGH)
    #     else:
    #         DC.GPIO.output(self.driverx.pin_d,DC.GPIO.LOW)
    #     if Vy>0:
    #         DC.GPIO.output(self.drivery.pin_d,DC.GPIO.HIGH)
    #     else:
    #         DC.GPIO.output(self.drivery.pin_d,DC.GPIO.LOW)
        
    #     if Draw:
    #         self.Draw()
    #     if f1x:
    #         self.driverx.pul.ChangeFrequency(f1x)
    #         self.driverx.pul.start(50)
    #     else:
    #         self.driverx.pul.stop()
            
    #     if f1y:
    #         self.drivery.pul.ChangeFrequency(f1y)
    #         self.drivery.pul.start(50)
    #     else:
    #         self.drivery.pul.stop()
        
    #     time.sleep(t-0.0017)
    #     self.driverx.pul.stop()
    #     if f2y:
    #         DC.GPIO.output(self.drivery.pin_d,DC.GPIO.LOW)
    #         self.drivery.pul.ChangeFrequency(f2y)
    #     else:
    #         self.drivery.pul.stop()
    #     if Draw:
    #         self.StopDraw()
def test1(): 
    GMmachine=GlueMasterMachine(modbusport='/dev/ttDriverXYZyUSB0', modbusbaudrate=115200,beltspeed=0)
    time.sleep(1)
    GMmachine.UpdatePosNow()
    GMmachine.Move2Pos([10 , -10,40],t=3,Draw=0)
    # time.sleep(1)
    # GMmachine.UpdatePosNow()
    # GMmachine.Move2Pos([-10,-10,0],t=2,Draw=1)
    # time.sleep(1)
    # GMmachine.UpdatePosNow()
    # GMmachine.DrawCircle([10,10],4,0)
    # time.sleep(1)
    # GMmachine.UpdatePosNow()
    del GMmachine
    # rad=2*math.pi*251/1000 
    # print(round(abs(10*math.cos(rad))/0.01))
    # print(0.0==0)
def main(): 
    GMmachine=GlueMasterMachine(modbusport='/dev/DriverXYZ', modbusbaudrate=115200,beltspeed=0)
    GMmachine.Move2Pos_Absolute(GMmachine.Rec_roughPos,wait=1,tip='test')
    GMmachine.UpdatePosNow()
    GMmachine.Move2Pos_Relative(GMmachine.Rec_rough2accuratePos,wait=1,tip='test2')
    GMmachine.UpdatePosNow()
    del GMmachine

if __name__=='__main__':
    test1()
