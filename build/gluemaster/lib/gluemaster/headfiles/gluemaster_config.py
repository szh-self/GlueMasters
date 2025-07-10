import csv
import cv2
import os
import subprocess
import re
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

RecVy=5#等待识别时y +向速度mm/s

# RecHeight_accurate=60#等待精识别时胶头离传送带的高度(mm)
# Recrough2AccurateX=10#精识别镜头相对于粗识别镜头的x距离(mm)
# Recrough2AccurateY=10#精识别镜头相对于粗识别镜头的y距离(mm)
# Accurate2RealX=10#针头相对于精识别镜头的x距离(mm)
# Accurate2RealY=10#针头相对于精识别镜头的y距离(mm)
CatchUpProductTime=1#针头在已经和传送带共速的情况下追到产品中心点的时间
# Zpos0=-7200#针头碰到传送带时的z坐标p
gluemasterfilepath='./gluemaster_files'
# ruler_rough=[1,1]#像素/mm
# ruler_accurate=1#像素/mm
HoFindV_H=30#寻找homeswitch时的速度（高）mm/s
HoFindV_L=5#寻找homeswitch时的速度（低）mm/s
#checkcodes
CHECK_COIN=0x01
CHECK_HOMESWITCH=0x08
CHECK_HOMEATTAIN=0x04
def Pixel2Real(pixel_pos,flag=0):#flag=1时表示精匹配ruler
    rulerdata=np.load(gluemasterfilepath+'/rulerdata.npz')
    if flag:
        ruler=rulerdata['ruler_accurate']
    else:
        ruler=rulerdata['ruler_rough']
    realposx=pixel_pos[0]/ruler[0]
    realposy=pixel_pos[1]/ruler[1]
    return (realposx,realposy)

def GetTypeAndTpl(flag=0,SpecificType=None):#flag=1时取精匹配模板 返回type和tpl列表
    try:
        path=gluemasterfilepath + '/connections.csv'
        with open(path, 'r') as csvfile:
            type=[]
            tplpath=[]
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                if row[0]=='1':
                    if SpecificType:
                        if row[1]==SpecificType:
                            type.append(row[1])
                            tplpath.append(row[2+flag])
                            return type,tplpath
                    else:
                        type.append(row[1])
                        tplpath.append(row[2+flag])
            print('Get All template ',type,' ',tplpath)
            return type,tplpath
    except FileNotFoundError:
        print(path," File is not found.")
    except PermissionError:
        print(path," You don't have permission to access this file.")

def WriteCSVlist(data,path=gluemasterfilepath + '/camparam.csv'):
    with open(path, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data)

def GetCSVlist(path=gluemasterfilepath + '/camparam.csv'):
    csvlist=[]
    with open(path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        for row in csv_reader:
            temp=[]
            for i in range(len(row)):
                temp.append(eval(row[i]))
            csvlist.append(temp)
    return csvlist

def kill_processes_using_tty(device="/dev/ttyUSB0"):
    """
    杀死所有占用指定串口设备(如/dev/ttyUSB0)的进程
    
    参数:
        device (str): 要释放的串口设备路径，默认为/dev/ttyUSB0
    
    返回:
        tuple: (成功杀死的进程数量, 错误信息)
    """
    try:
        # 1. 检查设备是否存在
        if not os.path.exists(device):
            return 0, f"错误：设备 {device} 不存在"
        
        # 2. 使用 lsof 查找占用设备的进程
        result = subprocess.run(
            ['lsof', '-t', device],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 3. 处理可能的错误
        if result.returncode != 0:
            if "No such file or directory" in result.stderr:
                return 0, f"错误：找不到命令 'lsof'，请先安装: sudo apt install lsof"
            elif "Permission denied" in result.stderr:
                return 0, f"错误：权限不足，请使用sudo运行此命令"
            elif "no process found" in result.stderr.lower():
                return 0, "没有进程占用该设备"
            else:
                return 0, f"未知错误: {result.stderr.strip()}"
        
        # 4. 提取进程ID
        pids = result.stdout.strip().split()
        if not pids:
            return 0, "没有进程占用该设备"
        
        # 5. 杀死所有相关进程
        killed_count = 0
        errors = []
        
        for pid in pids:
            if not re.match(r'^\d+$', pid):
                continue  # 跳过无效PID
                
            try:
                # 向进程发送SIGTERM信号（正常终止）
                subprocess.run(['kill', '-TERM', pid], check=True)
                killed_count += 1
            except subprocess.CalledProcessError as e:
                errors.append(f"无法终止进程 {pid}: {e}")
        # 6. 返回结果
        if errors:
            return killed_count, " | ".join(errors)
        return killed_count, f"成功终止了 {killed_count} 个进程"
    
    except Exception as e:
        return 0, f"意外错误: {str(e)}"
    
def rotate_vector(v, angle):#v :nparray
    angle_rad=math.radians(angle)
    rotation_matrix = np.array([
        [math.cos(angle_rad), -math.sin(angle_rad)],
        [math.sin(angle_rad), math.cos(angle_rad)]
    ])
    return np.dot(rotation_matrix, v)

def VectorAxisCam2Real(Vector,flag):#Vector nparray
    if flag==1:
        # Vector1=np.array([Vector[0],-Vector[1]])
        Vector1=np.array([Vector[1],Vector[0]])
        # Vector1=rotate_vector(Vector1,-90)
    else:
        Vector1=np.array([Vector[0],-Vector[1]])
    return Vector1


# ROS 2 QoS配置示例（低CPU开销）
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1,
)