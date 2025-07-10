import cv2
import subprocess
import os
import signal
import time
import gluemaster.headfiles.gluemaster_config as gm_config

class CameraSystem:
    def __init__(self):
        # 初始化虚拟设备
        # self.setup_virtual_device()
        self.fallback_mode = True
        # 创建OpenCV捕获对象（始终使用虚拟设备）
        # 启动初始转发进程（默认精确摄像头）
        self.forward_pid = None
        self.CurrentCam = False  # True表示精确摄像头
        # self.start_forwarding("/dev/CamRough")
        self.cap = cv2.VideoCapture("/dev/CamRough", cv2.CAP_V4L2)
        self.ChangeCam(AccurateFlag=False, ForceResetFlag=True)
    def setup_virtual_device(self):
        """确保虚拟设备存在"""
        try:
            # 检查虚拟设备是否存在
            if not os.path.exists("/dev/video10"):
                # 创建虚拟设备
                subprocess.run([
                    "sudo", "modprobe", "v4l2loopback", 
                    "devices=1", "video_nr=10", "card_label=CamActive",
                    "exclusive_caps=1"
                ], check=True)
            
            # 确保虚拟设备可访问
            if not os.access("/dev/video10", os.R_OK | os.W_OK):
                subprocess.run(["sudo", "chmod", "777", "/dev/video10"], check=True)
                
            if not os.path.exists("/dev/CamActive"):
                subprocess.run([
                    "sudo", "ln", "-sf", "/dev/video10", "/dev/CamActive"
                ], check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"虚拟设备设置失败: {e}")
            # 回退到直接使用物理设备
            self.fallback_mode = True
    
    def start_forwarding(self, physical_device):
        """启动视频流转发到虚拟设备"""
        # 停止现有转发进程
        self.stop_forwarding()
        
        # 启动新的转发进程（低延迟模式）
        command = [
            "ffmpeg",
            "-f", "v4l2",
            "-use_wallclock_as_timestamps", "1",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-avioflags", "direct",
            "-i", physical_device,
            "-c:v", "copy",  # 直接复制流，不重新编码
            "-f", "v4l2",
            "/dev/video10",
            "-loglevel", "error"  # 减少日志输出
        ]
        
        process = subprocess.Popen(command)
        self.forward_pid = process.pid
        print(f"启动转发: {physical_device} → /dev/video10 (PID: {self.forward_pid})")
        
        # 等待流稳定
        time.sleep(0.05)
        
    def stop_forwarding(self):
        """停止视频流转发"""
        if self.forward_pid:
            try:
                os.kill(self.forward_pid, signal.SIGTERM)
                print(f"停止转发进程 PID: {self.forward_pid}")
            except ProcessLookupError:
                pass  # 进程已退出
            self.forward_pid = None
    
    def ChangeCam(self, AccurateFlag, ForceResetFlag=True):
        """切换摄像头（使用v4l2loopback优化）"""
        if not hasattr(self, 'fallback_mode'):
            # 使用v4l2loopback优化路径
            if ForceResetFlag or (AccurateFlag != self.CurrentCam):
                # 选择物理设备
                physical_device = "/dev/CamAccurate" if AccurateFlag else "/dev/CamRough"
                # 切换视频流源
                self.start_forwarding(physical_device)
        else:
            if ForceResetFlag or (AccurateFlag != self.CurrentCam):
                # 回退模式：直接切换物理设备
                self.cap.release()
                if AccurateFlag:
                    self.cap = cv2.VideoCapture("/dev/CamAccurate", cv2.CAP_V4L2)
                else:
                    self.cap = cv2.VideoCapture("/dev/CamRough", cv2.CAP_V4L2)
        if ForceResetFlag and hasattr(self,'cap'):# 设置物理摄像头参数
            self.set_opencv_camera_params(AccurateFlag)
        self.CurrentCam = AccurateFlag
    def get_frame(self):
        if hasattr(self,'cap'):
            ret,frame=self.cap.read()
            if ret:
                return ret,frame
        return False,None
            
    # def set_physical_camera_params(self, AccurateFlag, device):
    #     """使用v4l2-ctl设置物理摄像头参数"""
    #     try:
    #         path = gm_config.gluemasterfilepath + '/camparam.csv'
    #         camsetlistall = gm_config.GetCSVlist(path=path)
    #         camsetlist = camsetlistall[AccurateFlag]
    #         print(f'设置 {device} 参数:', camsetlist)
            
    #         # 使用v4l2-ctl设置参数
    #         v4l2_cmd = ["v4l2-ctl", "-d", device]
            
    #         # 设置格式和帧率
    #         subprocess.run(v4l2_cmd + [
    #             "--set-fmt-video", 
    #             f"width={camsetlist[1]},height={camsetlist[2]}"
    #         ], check=True)
            
    #         subprocess.run(v4l2_cmd + [
    #             "--set-parm", str(camsetlist[0])
    #         ], check=True)
            
    #         # 设置自动对焦
    #         if camsetlist[3] == 0:
    #             subprocess.run(v4l2_cmd + ["-c", "focus_auto=0"], check=True)
    #             subprocess.run(v4l2_cmd + ["-c", f"focus_absolute={camsetlist[4]}"], check=True)
    #         else:
    #             subprocess.run(v4l2_cmd + ["-c", "focus_auto=1"], check=True)
            
    #         # 设置曝光
    #         if camsetlist[5] == 1:
    #             subprocess.run(v4l2_cmd + ["-c", "exposure_auto=3"], check=True)
    #         else:
    #             subprocess.run(v4l2_cmd + ["-c", "exposure_auto=1"], check=True)
    #             subprocess.run(v4l2_cmd + ["-c", f"exposure_absolute={camsetlist[6]}"], check=True)
                
    #     except FileNotFoundError:
    #         print(path, "CamParam File is not found.")
    #     except PermissionError:
    #         print(path, "You don't have permission to access this CamParam File.")
    #     except subprocess.CalledProcessError as e:
    #         print(f"v4l2-ctl设置失败: {e}")
    
    def set_opencv_camera_params(self, AccurateFlag):
        """设置OpenCV摄像头参数（回退模式使用）"""
        try:
            path = gm_config.gluemasterfilepath + '/camparam.csv'
            camsetlistall = gm_config.GetCSVlist(path=path)
            camsetlist = camsetlistall[AccurateFlag]
            print('Cam set param :', camsetlist)
            
            self.cap.set(cv2.CAP_PROP_FPS, camsetlist[0])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camsetlist[1])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camsetlist[2])
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, camsetlist[3])
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, camsetlist[5])
            
            if camsetlist[3] == 0:
                self.cap.set(cv2.CAP_PROP_FOCUS, camsetlist[4])
            if camsetlist[5] == 1:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, camsetlist[6])
                
        except FileNotFoundError:
            print(path, "CamParam File is not found.")
        except PermissionError:
            print(path, "You don't have permission to access this CamParam File.")
    
    def __del__(self):
        """清理资源"""
        self.stop_forwarding()
        if hasattr(self, 'cap'):
            self.cap.release()