#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
from example_interfaces.srv import AddTwoInts
from example_interfaces.srv import SetBool
from gm_interfaces.srv import Str2Str
from std_msgs.msg import Float32
import gluemaster.headfiles.gluemaster_config as gm_config
import gluemaster.headfiles.Match_template_VIII as Match_template_VIII
import gluemaster.headfiles.Demarcate as Demarcate
from gluemaster.headfiles.Camsys import CameraSystem
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import numpy as np
import math
class Rec_Node(Node):
    def __init__(self):
        super().__init__('Rec_Node')
        Srv_group = MutuallyExclusiveCallbackGroup()
        Publisher_group = MutuallyExclusiveCallbackGroup()
        Timer_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger,"RecgSrv_rough",self.RecSrv_roughCallBack,callback_group=Srv_group)
        self.create_service(Trigger,"RecgSrv_accurate",self.RecSrv_accurateCallBack,callback_group=Srv_group)
        self.create_service(AddTwoInts,"TestRecSrv",self.TestRecSrvCallBack,callback_group=Srv_group)
        self.create_service(AddTwoInts,"DemarcateSrv",self.DemarcateSrvCallBack,callback_group=Srv_group)
        self.create_service(Str2Str,"RecAccurateWithRoughResult",self.RecAccurateWithRoughResultCallBack,callback_group=Srv_group)
        self.create_service(AddTwoInts,"Change_Using_cam",self.Change_Using_camCallBack,callback_group=Srv_group)
        self.Imagepublisher = self.create_publisher(Image,"Original_Image", callback_group=Publisher_group,qos_profile=gm_config.qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback,callback_group=Timer_group)
        self.bridge = CvBridge()
        self.cam_system = CameraSystem()
        # while not hasattr(self.cam_system,"cap"):
        #     self.cam_system.cap = cv2.VideoCapture("/dev/CamActive")
        #     if not self.cam_system.cap.isOpened():
        #         del self.cam_system.cap
        #     time.sleep(1)
    def ChangeCam(self,AccurateFlag,ForceResetFlag=True):#has release
        time1=time.time()
        self.cam_system.ChangeCam(AccurateFlag,ForceResetFlag=True)
        print('Change Cam taken time:',time.time()-time1)
    # def ChangeCam(self,AccurateFlag,ForceResetFlag=True):#No release
    #     if ForceResetFlag:
    #         if AccurateFlag:
    #             self.cap=self.capaccurate
    #         else:
    #             self.cap=self.caprough
    #         try:
    #             path=gm_config.gluemasterfilepath + '/camparam.csv'
    #             camsetlistall=gm_config.GetCSVlist(path=path)
    #             camsetlist=camsetlistall[AccurateFlag]
    #             print('Cam set param :',camsetlist)
    #             self.cap.set(cv2.CAP_PROP_FPS, camsetlist[0])
    #             self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camsetlist[1])
    #             self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camsetlist[2])
    #             self.cap.set(cv2.CAP_PROP_AUTOFOCUS, camsetlist[3])#自动对焦
    #             self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, camsetlist[5])#自动曝光
    #             # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 1)#亮度 1
    #             # self.cap.set(cv2.CAP_PROP_CONTRAST,40)#对比度 40
    #             # self.cap.set(cv2.CAP_PROP_SATURATION, 50)#饱和度 50
    #             # self.cap.set(cv2.CAP_PROP_HUE, 50)#色调 50
    #             if camsetlist[3]==0:
    #                 self.cap.set(cv2.CAP_PROP_FOCUS, camsetlist[4])#焦距
    #             if camsetlist[5]==1:
    #                 self.cap.set(cv2.CAP_PROP_EXPOSURE, camsetlist[6])#曝光 50
                
    #         except FileNotFoundError:
    #             print(path,"CamParam File is not found.")
    #         except PermissionError:
    #             print(path," You don't have permission to access this CamParam File.")
    #     else:
    #         if AccurateFlag == self.CurrentCam:
    #             pass
    #         else:
    #             if AccurateFlag:
    #                 self.cap=self.capaccurate
    #             else:
    #                 self.cap=self.caprough
    #     self.CurrentCam=AccurateFlag

    def RecSrv_roughCallBack(self,request,response):
        self.ChangeCam(AccurateFlag=0,ForceResetFlag=False)
        results,boxed_img = self.RecImage(flag=0)
        if results:
            response.success = True
            response.message=results
        else:
            response.success=False
            response.message="None"
        return response
    
    def RecSrv_accurateCallBack(self,request,response):
        self.ChangeCam(AccurateFlag=1,ForceResetFlag=False)
        results,boxed_img = self.RecImage(flag=1)
        if results:
            response.success = True
            response.message=results
        else:
            response.success=False
            response.message="None"
        return response
    
    def RecAccurateWithRoughResultCallBack(self,request,response):
        message=eval(request.inputmessage)
        print(message)
        angle_range_min=message[0]
        angle_range_max=message[1]
        accuratetype=message[2]
        self.ChangeCam(AccurateFlag=1,ForceResetFlag=False)
        # results,boxed_img = self.RecImage(flag=1,angle_range_min=angle_range_min,angle_range_max=angle_range_max,SpecificType=accuratetype,returnbox=1)
        results,boxed_img = self.RecImage(flag=1,angle_range_min=angle_range_min,angle_range_max=angle_range_max,SpecificType=accuratetype)
        if results:
            response.success = True
            response.outputmessage=results
            # cv2.imshow('RecAccurateWithRoughResult Rec Result', boxed_img)
            # cv2.namedWindow('RecAccurateWithRoughResult Rec Result', cv2.WINDOW_AUTOSIZE)
            # cv2.waitKey(10000)
            # cv2.destroyAllWindows()
        else:
            response.success=False
            response.outputmessage="None"
        return response
    
    def Change_Using_camCallBack(self,request,response):
        if request.a:
            # tempstr='accurate'
            self.ChangeCam(AccurateFlag=1,ForceResetFlag=bool(request.b))
        else:
            # tempstr='rough'
            self.ChangeCam(AccurateFlag=0,ForceResetFlag=bool(request.b))
        response.sum = 1
        # print(response.message)
        return response
    def TestRecSrvCallBack(self,request,response):
        if request.a:
            results,boxed_img = self.RecImage(flag=request.a,returnbox=1)
            if results:
                cv2.imshow('Accurate Rec Result', boxed_img)
                cv2.namedWindow('Accurate Rec Result', cv2.WINDOW_AUTOSIZE)
                cv2.waitKey(10000)
                cv2.destroyAllWindows()
                response.sum=1
            else:
                response.sum=0
        else:
            results,boxed_img = self.RecImage(flag=request.a,returnbox=1)
            if results:
                cv2.imshow('Rough Rec Result', boxed_img)
                cv2.namedWindow('Rough Rec Result', cv2.WINDOW_AUTOSIZE)
                cv2.waitKey(10000)
                cv2.destroyAllWindows()
                response.sum=1
            else:
                response.sum=0
        return response
    
    def DemarcateSrvCallBack(self,request,response):
        ret, frame = self.cam_system.get_frame()
        # cv2.imshow('origin demarcate',frame)
        # cv2.imwrite(gm_config.gluemasterfilepath+'/waa.png',frame)
        response.sum=0
        if ret:
            rulerx,rulery=Demarcate.calculate_pixel_per_mm([frame], square_size_cm=request.a/1000)
            if not rulerx:
                rulerx=0.01
            if not rulery:
                rulery=0.01
            print('rulerx',rulerx)
            print('rulery',rulery)
            with np.load(gm_config.gluemasterfilepath+'/rulerdata.npz') as data:
                # 2. 将数据提取到字典中
                data_dict = {key: data[key] for key in data.files}

                # 3. 修改特定的数组
                if request.b:
                    data_dict['ruler_accurate']=np.array([rulerx,rulery])
                else:
                    data_dict['ruler_rough']=np.array([rulerx,rulery])
                    
                # 4. 保存修改后的数据
                np.savez(gm_config.gluemasterfilepath+'/rulerdata.npz', **data_dict)
                response.sum=1
        return response
    
    def timer_callback(self):
        
        # if hasattr(self.cam_system,'cap'):
        ret, frame = self.cam_system.get_frame()
        
        if ret:
            # print("timer_start")
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # self.get_logger().info('Publishing: %f' % 1)
            self.Imagepublisher.publish(msg)##发布的暂时是原始图像
        else:
            print('未发送图像')
    
    def RecImage(self,flag=0,angle_range_min=0,angle_range_max=360,returnbox=0,SpecificType=None):##识别函数返回一个字符串flag=1表示进行精匹配
        ret, frame = self.cam_system.get_frame()
        h, w = frame.shape[:2]
        CenterPos=np.array([int(w/2),int(h/2)])
        self.timer.reset()
        message=[]
        target_type,tplpath=gm_config.GetTypeAndTpl(flag=flag,SpecificType=SpecificType)#获取类型和模板列表eg:type[0]这个产品类型对应模板tpl[0]
        if flag and SpecificType:
            tpl=cv2.imread(tplpath[0])
            tplh, tplw = tpl.shape[:2]
            resizelen=int(min(min(h,w),1.7*max(tplh,tplw)))
            frame=frame[int(h/2-resizelen/2):int(h/2+resizelen/2),int(w/2-resizelen/2):int(w/2+resizelen/2)]
            CenterPos=np.array([int(resizelen/2),int(resizelen/2)])
        all_results, boxed_img = Match_template_VIII.recognize_all_signs(frame,tplpath,target_type , flag=flag,angle_range=(angle_range_min, angle_range_max),returnbox=returnbox)
        if len(all_results)>0:
            for result in all_results:
                print('PixelRecResult',result)
                Vector0=np.array([result[1],result[2]])
                Vector1=Vector0-CenterPos
                Vector1=gm_config.VectorAxisCam2Real(Vector1,flag)
                tempresult=gm_config.Pixel2Real(Vector1.tolist(),flag=flag)
                product=(result[0],tempresult[0],tempresult[1],result[3])
                print('RealRecResult:',product)
                message.append(product)
            message=str(tuple(message))
        else:
            message=None
        Timer_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.timer_callback,callback_group=Timer_group)
        return message ,boxed_img #message="((type,x,y,angle),(type,x,y,angle))..."x,y为产品中心点相对于针头的距离mm
    def __del__(self):
        del self.cam_system
        
    
def main(args=None):
    rclpy.init(args=args)

    Rec_Con = Rec_Node()
    rclpy.spin(Rec_Con)

    Rec_Con.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()
