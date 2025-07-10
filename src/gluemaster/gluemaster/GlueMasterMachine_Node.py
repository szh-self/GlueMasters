#!/usr/bin/env python3

import gluemaster.headfiles.GlueMasterMachine_Con as GM_Con
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
from example_interfaces.srv import AddTwoInts
from example_interfaces.srv import SetBool
from gm_interfaces.srv import Str2Str
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import csv
import gluemaster.headfiles.gluemaster_config as gm_config
from pynput import keyboard
from threading import Thread
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import numpy as np
class GlueMaster_Node(Node):
    def __init__(self,gluemasterfilepath):
        super().__init__('GlueMaster_Node')
        gm_config.kill_processes_using_tty(device="/dev/DriverXYZ")
        self.GMmachine=GM_Con.GlueMasterMachine(modbusport='/dev/DriverXYZ', modbusbaudrate=115200,beltspeed=0)
        Srv_group = MutuallyExclusiveCallbackGroup()
        Subscriber_group = MutuallyExclusiveCallbackGroup()
        Client_group = MutuallyExclusiveCallbackGroup()
        Timer_group = MutuallyExclusiveCallbackGroup()
        Publisher_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger,"ShutDownSrv",self.ShutDownSrvCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"StartSrv",self.StartSrvCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"ResetSrv",self.ResetSrvCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"CalibrationModeSrv",self.CalibrationModeSrvCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"Move2RecRoughPosSrv",self.Move2RecRoughPosSrvCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"Move2RecAccuratePosSrv",self.Move2RecAccuratePosSrvCallBack,callback_group=Srv_group)
        # self.create_service(Trigger,"DemarcateSrv",self.DemarcateSrvCallBack,callback_group=Srv_group)
        self.create_subscription(Float32,'belt_speed_topic',self.listen_beltspeed_callback,callback_group=Subscriber_group,qos_profile=gm_config.qos_profile)
        self.CalibrationFlagpublisher = self.create_publisher(Int32,"CalibrationFlag",callback_group=Publisher_group,qos_profile=gm_config.qos_profile)
        self.RecRoughClient = self.create_client(Trigger, 'RecgSrv_rough',callback_group=Client_group)
        self.RecAccurateClient = self.create_client(Trigger, 'RecgSrv_accurate',callback_group=Client_group)
        self.RecAccurateWithRoughResultClient = self.create_client(Str2Str, 'RecAccurateWithRoughResult',callback_group=Client_group)
        # self.RecDemarcateClient = self.create_client(Trigger, 'RecDemarcateSrv',callback_group=Client_group)
        self.Change_Using_camClient = self.create_client(AddTwoInts, 'Change_Using_cam',callback_group=Client_group)
        self.Rec_ChangeAngleRangeClient = self.create_client(AddTwoInts, 'Change_angle_range',callback_group=Client_group)
        if not self.RecAccurateWithRoughResultClient.wait_for_service(timeout_sec=4): 
            raise RuntimeError('RecAccurateWithRoughResult not available')
        # self.NextUI_CalibrationClient = self.create_client(AddTwoInts, 'Next_calibration',callback_group=Client_group)
        if not self.RecRoughClient.wait_for_service(timeout_sec=4):
                    raise RuntimeError('RecgSrv_rough service not available')
        if not self.RecAccurateClient.wait_for_service(timeout_sec=4):
            raise RuntimeError('RecgSrv_accurate service not available')
        if not self.Change_Using_camClient.wait_for_service(timeout_sec=8): 
            raise RuntimeError('Change_Using_cam service not available')
        # if not self.RecDemarcateClient.wait_for_service(timeout_sec=8): 
        #     raise RuntimeError('RecDemarcateSrv service not available')
        # if not self.NextUI_CalibrationClient.wait_for_service(timeout_sec=8): 
        #     raise RuntimeError('Next_calibration service not available')
        self.status='IDLE'
        self.statusupdatetimer=self.create_timer(0.1,self.UpdateStatus,callback_group=Timer_group)
        self.CurrentProducts=[]#用于记录已经处理的产品，记录第一次识别添加时的绝对坐标和时间，类型及角度
        self.ProcessedProductNum=0
        self.gluemasterfilepath=gluemasterfilepath
        self.angle_range_min=0#识别的产品角度范围，用于粗识别后加速精识别
        self.angle_range_max=360
    
    
    def ShutDownSrvCallBack(self,request,response):
        self.GMmachine.ShutDown()
        self.status='IDLE'
        response.success = True
        response.message = "The machine has been shutdown"
        print(response.message)
        return response
    def StartSrvCallBack(self,request,response):
        self.GMmachine.Start()
        self.status='WORKING_WAITING'
        self.GMmachine.Move2Pos_Absolute(Pos_p=self.GMmachine.Rec_roughPos,wait=1,tip='Move to Rec_accurate Pos')
        response.success = True
        response.message = "The machine has been started"
        print(response.message)
        return response
        
    def ResetSrvCallBack(self,request,response):
        self.GMmachine.Reset()
        self.status='IDLE'
        response.success = True
        response.message = "The machine has been Reset"
        print(response.message)
        return response
        
    def Move2RecAccuratePosSrvCallBack(self,request,response):
        self.GMmachine.Start()
        self.status='IDLE'
        self.GMmachine.Move2Pos_Absolute(Pos_p=self.GMmachine.Rec_roughPos,wait=1,tip='Move to Rec_accurate Pos1')
        self.GMmachine.Move2Pos_Relative(Pos_p=self.GMmachine.Rec_rough2accuratePos,wait=1,tip='Move to Rec_accurate Pos2')
        response.success = True
        response.message = "The machine has been set to Rec_accurate Pos"
        print(response.message)
        return response
    
    def Move2RecRoughPosSrvCallBack(self,request,response):
        self.GMmachine.Start()
        self.status='IDLE'
        self.GMmachine.Move2Pos_Absolute(Pos_p=self.GMmachine.Rec_roughPos,wait=1,tip='Move to Rec_rough Pos')
        response.success = True
        response.message = "The machine has been set to Rec_rough Pos"
        print(response.message)
        return response
    
    def CalibrationModeSrvCallBack(self,request,response):
        self.GMmachine.Start()
        self.status='IDLE'
        self.GMmachine.Move2Pos_Absolute(Pos_p=self.GMmachine.Rec_roughPos,wait=1,tip='Move to Rec_rough Pos')
        self.CalibrationModeInitialize()
        response.success = True
        response.message = "The machine has been set to CalibrationMode"
        print(response.message)
        return response

    # def DemarcateSrvCallBack(self,request,response):
    #     self.GMmachine.Start()
    #     self.status='IDLE'
    #     self.GMmachine.Move2Pos_Absolute(Pos_p=self.GMmachine.Rec_roughPos,wait=1,tip='Move to get rough tpl')
    #     self.SetCam(choose=0)
    #     response.success = True
    #     response.message = "The machine has been set to get ready for rough template"
    #     print(response.message)
    #     return response

    def listen_beltspeed_callback(self,msg):
        self.GMmachine.beltspeed=msg.data
        # print('beltspeednow:',self.GMmachine.beltspeed)
        
    def __HasSameProduct(self,data): #data=(type,x,y(绝对),angle,time)
        IsSame=False
        for addedproduct in self.CurrentProducts:#addedproduct=(type,x,y(绝对),ang,time)
            if (addedproduct[0]==data[0]):
                deldata=np.abs(np.array(addedproduct[1:])-np.array(data[1:]))
                print('deldata',deldata)
                # if (deldata[2]<=2 or deldata[2]>=358) and deldata[0]<=10 and abs(deldata[1]-deldata[3]*self.GMmachine.beltspeed)<=10:
                if deldata[0]<=10 and abs(deldata[1]-deldata[3]*self.GMmachine.beltspeed)<=10:
                    IsSame=True
        return IsSame
    
    def AddCurrentProduct(self,product):#product=(type,x,y(相对),ang)
        # "(-9240, -607, -6292)","(613, -8835, 4)","(-5026, 12385, -7212)"
        if len(self.CurrentProducts) >= 5:
            del self.CurrentProducts[0]
        temp=list(product)#添加第一次粗识别时的绝对坐标()和时间
        temp.append(time.time())
        self.GMmachine.UpdatePosNow()
        for j in [1,2]:
            temp[j]=temp[j]+self.GMmachine.Pos[j-1]*0.01
        self.CurrentProducts.append(temp)#temp=(type,x,y(绝对),ang,time)
        # "(-9240, -607, -6292)","(613, -8835, 4)"
    def __FindProcessingProduct(self,message,accurateflag):#找到第一个未处理的产品,成功时返回product(message中的某个)
        #message="((type,x(相对),y,ang),(type,x,y,ang)...)"
        products = eval(message)
        for product in products:
            if accurateflag and product[0]==self.CurrentProducts[-1][0]:
                return product
            else:
                temp=list(product)
                temp.append(time.time())
                self.GMmachine.UpdatePosNow()
                for j in [1,2]:
                    temp[j]=temp[j]+self.GMmachine.Pos[j-1]*0.01
                if not self.__HasSameProduct(temp):
                    return product
        return False
    
    def GetCSVMethodPath(self,type):
        try:
            path=self.gluemasterfilepath + '/connections.csv'
            with open(path, 'r',encoding='utf-8') as csvfile:
                csv_reader = csv.reader(csvfile)
                for row in csv_reader:
                    if (row[0]=='1')and(row[1]==type):
                        print('Found ',type,' CSV')
                        return row[4]
            return None   
        except FileNotFoundError:
            print(path," File is not found.")
        except PermissionError:
            print(path," You don't have permission to access this file.")
    
    def SetCam(self,choose,ForceResetFlag=1):
        if not self.Change_Using_camClient.wait_for_service(timeout_sec=4): 
            raise RuntimeError('Change_Using_cam service not available')
        request = AddTwoInts.Request()
        request.a=choose
        request.b=ForceResetFlag
        future = self.Change_Using_camClient.call_async(request)

    def KeylistenerInitialize(self):
        self.keylistener=keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener_thread = Thread(target=self.keylistener.start)
        
    def CalibrationSendUIrequest(self):
        msg = Int32()
        msg.data = self.CalibrationFlag
        self.CalibrationFlagpublisher.publish(msg)
        print('msg has sent')
        self.CalibrationFlag=self.CalibrationFlag+1
        
        
    def CalibrationModeInitialize(self):
        self.CalibrationV=1#(mm/s)
        self.CalibrationFlag=0#校准步骤数
        self.current_keys = set()
        self.CalibrationData=[]
        self.KeylistenerInitialize()
        self.listener_thread.start()
        self.GMmachine.driverz.ShutDown()
        self.GMmachine.driverz.SpeedUp(0)
        self.GMmachine.driverz.SpeedModeSet()
        self.GMmachine.driverz.Start()
        self.SetCam(choose=0)
        self.CalibrationSendUIrequest()
        print('ok ini')
        
    def ExitCalibrationMode(self):
        self.CalibrationV=1#(mm/s)
        self.current_keys = set()
        self.CalibrationFlag=0#校准步骤数
        self.keylistener.stop()
        self.KeylistenerInitialize()
        self.GMmachine.driverz.ShutDown()
        self.GMmachine.driverz.SpeedUp(0)
        self.GMmachine.driverz.PositionModeSet(pulse=0)
        self.GMmachine.driverz.Start()
        self.SetCam(choose=0)
        self.GMmachine.ReadCalibrationData()
        print('Calibration Over')

    def on_press(self,key):
        self.current_keys.add(key)
        if key==keyboard.KeyCode(char='r'):
            self.CalibrationV=10#(mm/s)
        self.GMmachine.Key_response(v=self.CalibrationV,CalibrationV=self.CalibrationV,key=key,current_keys=self.current_keys)
        if key==keyboard.Key.enter:
            self.GMmachine.UpdatePosNow()
            self.CalibrationData.append(tuple(self.GMmachine.Pos))
            print('Real Pos: ',self.GMmachine.Pos)
            self.CalibrationSendUIrequest()
            if self.CalibrationFlag>3:
                temp=[]  #粗识别绝对位置，精识别镜头相对于粗识别镜头的x,y,z坐标和焦距,针头相对于精识别镜头的x,y,z坐标
                temp.append(self.CalibrationData[0])
                for i in range(len(self.CalibrationData)-1):
                    temp2=[]
                    for j in range(3):
                        temp2.append(self.CalibrationData[i+1][j]-self.CalibrationData[i][j])
                    temp.append(tuple(temp2))
                with open(self.gluemasterfilepath+'/CalibrationData.csv', mode='w', encoding='utf-8',newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(temp)  # 写单行
                self.ExitCalibrationMode()
            elif self.CalibrationFlag==2:
                    self.SetCam(choose=1)
                    self.GMmachine.Move2Pos_Relative(Pos_p=self.GMmachine.Rec_rough2accuratePos,wait=1,tip='Move to Rec_accurate Pos2')
            
    def on_release(self,key):
        try:
            self.current_keys.remove(key)
            if key==keyboard.KeyCode(char='r'):
                self.CalibrationV=1#(mm/s)
            self.GMmachine.Key_response(v=0,CalibrationV=self.CalibrationV,key=key,current_keys=self.current_keys)
        except KeyError:
            print('key error')

    def UpdateStatus(self):#状态机
        match self.status:
            case 'IDLE':
                pass
                # self.get_logger().info('is alive: %s' % self.listener_thread.is_alive())
            case 'WORKING_WAITING':#工作循环1 如果识别到，进入循环2，如果没识别到，+向移动直到-1000然后等待
                self.SetCam(choose=0,ForceResetFlag=0)
                self.GMmachine.SlowReturn(gm_config.RecVy,self.GMmachine.Rec_roughPos[1],self.GMmachine.Rec_roughPos[0],self.GMmachine.Rec_roughPos[2])
                request = Trigger.Request()
                timebeforeroughrec=time.time()
                self.GMmachine.UpdatePosNow()
                future = self.RecRoughClient.call_async(request)
                posybeforeroughrec = self.GMmachine.Pos[1]
                rclpy.spin_until_future_complete(self, future,timeout_sec=30)
                response = future.result()
                ProductFound=None
                if response.success:#如果识别到了(可能有多个)
                    #先判断有没有没处理过的
                    ProductFound=self.__FindProcessingProduct(response.message,accurateflag=0)
                    if  ProductFound:
                        self.GMmachine.UpdatePosNow()
                        posybeforecatchup = self.GMmachine.Pos[1]
                        dely=(posybeforeroughrec-posybeforecatchup)*0.01
                        self.GMmachine.CatchUpBelt()
                        deltime=time.time()-timebeforeroughrec
                        self.AddCurrentProduct(ProductFound)
                        self.SetCam(choose=1,ForceResetFlag=0)
                        dellength=-self.GMmachine.beltspeed*deltime
                        if self.GMmachine.SafetyCheck():
                            self.GMmachine.Move2Pos(Pos_mm=[ProductFound[1]+self.GMmachine.Rec_rough2accuratePos[0]*0.01,ProductFound[2]+self.GMmachine.Rec_rough2accuratePos[1]*0.01+dellength,self.GMmachine.Rec_accurateHeight],t=gm_config.CatchUpProductTime,Draw=0)
                            self.status='WORKING_PROCESSING'
                            # self.angle_range_min=(ProductFound[3]-5)%360
                            # self.angle_range_max=(ProductFound[3]+5)%360
                            self.angle_range_min=0
                            self.angle_range_max=360
                        else:
                            print('精识别将超程，放弃:',ProductFound[0])
                if self.status=='WORKING_PROCESSING':
                    self.get_logger().info('开始执行')
                else:
                    self.get_logger().info('等待识别中，...')
            case 'WORKING_PROCESSING':#工作循环2
                request = Str2Str.Request()
                request.inputmessage=str((self.angle_range_min,self.angle_range_max,self.CurrentProducts[-1][0]))
                future = self.RecAccurateWithRoughResultClient.call_async(request)
                rclpy.spin_until_future_complete(self, future,timeout_sec=30)
                response = future.result()
                ProductFound=None
                if response.success:
                    print('get RecAccurateWithRoughResult')
                    ProductFound=self.__FindProcessingProduct(response.outputmessage,accurateflag=1)
                if ProductFound:
                    self.GMmachine.Move2Pos(Pos_mm=[ProductFound[1]+self.GMmachine.Rec_accurate2realPos[0]*0.01,ProductFound[2]+self.GMmachine.Rec_accurate2realPos[1]*0.01,self.GMmachine.Rec_accurateHeight],t=gm_config.CatchUpProductTime,Draw=0)
                    CSVMethodPath=self.GetCSVMethodPath(type=ProductFound[0])
                    if CSVMethodPath:
                        self.GMmachine.GetCSVMethod(CSVMethodPath)
                    else:
                        raise RuntimeError('未找到此模板的点胶方式')
                    self.GMmachine.ExcecuteMethodWithAngle(angle=ProductFound[3])
                    self.status='WORKING_WAITING'
                    self.ProcessedProductNum=self.ProcessedProductNum+1
                    self.GMmachine.SlowReturn(gm_config.RecVy,self.GMmachine.Rec_roughPos[1],self.GMmachine.Rec_roughPos[0],self.GMmachine.Rec_roughPos[2])
                else:
                    print('精识别失败')
                    del self.CurrentProducts[-1]
                    self.status='WORKING_WAITING'
            case _:
                self.status='IDLE'
    def __del__(self):
        self.destroy_node()
        self.keylistener.stop()
def main(args=None):
    rclpy.init(args=args)

    GlueMaster_Con = GlueMaster_Node(gm_config.gluemasterfilepath)

    rclpy.spin(GlueMaster_Con)
    
    GlueMaster_Con.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()
