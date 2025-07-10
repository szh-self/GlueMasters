import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import signal
import time
import gluemaster.headfiles.gluemaster_config as gm_config
import subprocess
import cv2
import numpy as np
import threading
class CameraSystem:
    def __init__(self):
        Gst.init(None)
        self.CurrentCam = False  # True表示精确摄像头
        self.pipeline = None
        self.src = None
        self.create_pipeline()
        self.switch_camera("/dev/CamRough")  # 初始使用粗略摄像头

    def create_pipeline(self):
        """创建GStreamer管道"""
        self.pipeline = Gst.Pipeline.new("camera-pipeline")
        
        # 创建元素
        self.src = Gst.ElementFactory.make("v4l2src", "camera-source")
        self.capsfilter = Gst.ElementFactory.make("capsfilter", "caps-filter")
        self.videoconvert = Gst.ElementFactory.make("videoconvert", "converter")
        self.sink = Gst.ElementFactory.make("autovideosink", "display")
        
        # 设置初始caps（根据需求调整分辨率/帧率）
        caps = Gst.Caps.from_string("video/x-raw,width=640,height=480,framerate=30/1")
        self.capsfilter.set_property("caps", caps)
        
        # 添加元素到管道
        for element in [self.src, self.capsfilter, self.videoconvert, self.sink]:
            self.pipeline.add(element)
        
        # 链接元素
        self.src.link(self.capsfilter)
        self.capsfilter.link(self.videoconvert)
        self.videoconvert.link(self.sink)
        
        # 启动管道
        self.pipeline.set_state(Gst.State.PLAYING)

    def switch_camera(self, device_path):
        """动态切换摄像头设备"""
        # 暂停管道
        self.pipeline.set_state(Gst.State.PAUSED)
        
        # 修改v4l2src的设备属性
        self.src.set_property("device", device_path)
        
        # 重新播放
        self.pipeline.set_state(Gst.State.PLAYING)
        print(f"已切换摄像头: {device_path}")

    def ChangeCam(self, AccurateFlag, ForceResetFlag=True):
        """切换摄像头（精确/粗略）"""
        if ForceResetFlag or (AccurateFlag != self.CurrentCam):
            device = "/dev/CamAccurate" if AccurateFlag else "/dev/CamRough"
            self.switch_camera(device)
            self.CurrentCam = AccurateFlag
        if ForceResetFlag:
                self.set_camera_params(AccurateFlag)
    def set_camera_params(self, AccurateFlag):
        """设置摄像头参数（通过v4l2-ctl）"""
        try:
            path = gm_config.gluemasterfilepath + '/camparam.csv'
            camsetlistall = gm_config.GetCSVlist(path=path)
            camsetlist = camsetlistall[AccurateFlag]
            print(f'设置摄像头参数:', camsetlist)
            
            device = "/dev/CamAccurate" if AccurateFlag else "/dev/CamRough"
            v4l2_cmd = ["v4l2-ctl", "-d", device]
            
            # 设置分辨率
            subprocess.run(v4l2_cmd + [
                "--set-fmt-video", 
                f"width={camsetlist[1]},height={camsetlist[2]}"
            ], check=True)
            
            # 设置帧率
            subprocess.run(v4l2_cmd + [
                "--set-parm", str(camsetlist[0])
            ], check=True)
            
            # 设置对焦
            if camsetlist[3] == 0:
                subprocess.run(v4l2_cmd + ["-c", "focus_auto=0"], check=True)
                subprocess.run(v4l2_cmd + ["-c", f"focus_absolute={camsetlist[4]}"], check=True)
            else:
                subprocess.run(v4l2_cmd + ["-c", "focus_auto=1"], check=True)
            
            # 设置曝光
            if camsetlist[5] == 1:
                subprocess.run(v4l2_cmd + ["-c", "exposure_auto=3"], check=True)
            else:
                subprocess.run(v4l2_cmd + ["-c", "exposure_auto=1"], check=True)
                subprocess.run(v4l2_cmd + ["-c", f"exposure_absolute={camsetlist[6]}"], check=True)
                
        except FileNotFoundError:
            print(path, "CamParam File is not found.")
        except subprocess.CalledProcessError as e:
            print(f"v4l2-ctl设置失败: {e}")

    def __del__(self):
        """清理资源"""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            

class CameraSystemWithCapture(CameraSystem):
    def __init__(self):
        super().__init__()  # 继承基类初始化
        self.loop = GLib.MainLoop()
        self.bus = None
        self.frame_ready = False
        self.current_frame = None
        self.create_capture_pipeline()
        self.switch_camera("/dev/CamRough")
        
        # 添加延迟确保管道初始化完成
        time.sleep(1) 
        
        # 启动GLib主循环处理信号
        self.thread = threading.Thread(target=self.loop.run)
        self.thread.daemon = True
        self.thread.start()
    
    def create_capture_pipeline(self):
        """创建带appsink的管道"""
        # 先清理现有管道
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        
        # 创建新管道
        self.pipeline = Gst.Pipeline.new("capture-pipeline")
        
        # 创建元素
        self.src = Gst.ElementFactory.make("v4l2src", "source")
        self.capsfilter = Gst.ElementFactory.make("capsfilter", "filter")
        self.videoconvert = Gst.ElementFactory.make("videoconvert", "convert")
        self.sink = Gst.ElementFactory.make("appsink", "sink")
        
        # 配置appsink
        appsink_caps = Gst.Caps.from_string("video/x-raw,format=BGR")
        self.sink.set_property("caps", appsink_caps)
        self.sink.set_property("emit-signals", True)
        self.sink.set_property("sync", False)
        self.sink.set_property("max-buffers", 1)
        self.sink.set_property("drop", True)
        self.sink.connect("new-sample", self.on_new_sample)
        
        # 设置动态caps（不固定分辨率）
        caps = Gst.Caps.from_string("video/x-raw")
        self.capsfilter.set_property("caps", caps)
        
        # 构建管道
        self.pipeline.add(self.src)
        self.pipeline.add(self.capsfilter)
        self.pipeline.add(self.videoconvert)
        self.pipeline.add(self.sink)
        
        # 链接元素
        self.src.link(self.capsfilter)
        self.capsfilter.link(self.videoconvert)
        self.videoconvert.link(self.sink)
        
        # 设置总线监视器
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self.on_bus_message)
    
    def on_new_sample(self, appsink):
        """处理新帧回调"""
        sample = appsink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR
        
        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        
        try:
            # 动态获取帧尺寸
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_int("width").value
            height = structure.get_int("height").value
            
            # 创建帧副本（避免缓冲区问题）
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()
            
            self.current_frame = frame
            self.frame_ready = True
        finally:
            buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def on_bus_message(self, bus, message):
        """处理总线消息"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"GStreamer Error: {err}, Debug: {debug}")
        elif t == Gst.MessageType.EOS:
            print("End of Stream")
        return True
    
    def switch_camera(self, device_path):
        """完全重置管道来切换摄像头"""
        # 完全停止并重置管道
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            time.sleep(0.5)  # 确保完全停止
        
        # 设置新设备
        self.src.set_property("device", device_path)
        print(f"切换摄像头: {device_path}")
        
        # 重新创建管道
        self.create_capture_pipeline()
        
        # 启动管道
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("无法启动管道")
        time.sleep(1)  # 等待管道初始化
    
    def get_frame(self):
        """获取当前帧"""
        self.frame_ready = False
        return self.current_frame is not None,self.current_frame if self.current_frame is not None else None
    
    def __del__(self):
        """清理资源"""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop:
            self.loop.quit()