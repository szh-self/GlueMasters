from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt, QLineF, pyqtSignal, QThread, QPointF, QSize, QTimer, QEventLoop
from PyQt5.QtGui import QPen, QColor, QBrush, QPixmap, QImage, QPen, QPolygonF, QPainter, QIntValidator, QDoubleValidator, QPalette
from PyQt5.QtWidgets import (QGraphicsScene, QGraphicsLineItem, QComboBox, QTableWidgetItem, QTableWidget,
                            QGraphicsEllipseItem, QFileDialog, QColorDialog, QGraphicsPolygonItem,
                            QGraphicsPixmapItem, QDialog, QAbstractItemView, QHeaderView)
# from mainwindow import Ui_MainWindow
# from painter_set import Ui_Dialog_painter_set
# from mv_set import Ui_Dialog_mv_set
# from product_name import Ui_Dialog_product_name
# from tpl_cpt import Ui_Dialog_tpl_cpt
# from point_reset import Ui_Dialog_point_reset
# from parameter_reset import Ui_Dialog_parameter_reset
# from delete_accept import Ui_Dialog_delete_accept
# import gluemaster_config as gc
from gluemaster.headfiles.mainwindow import Ui_MainWindow
from gluemaster.headfiles.painter_set import Ui_Dialog_painter_set
from gluemaster.headfiles.mv_set import Ui_Dialog_mv_set
from gluemaster.headfiles.product_name import Ui_Dialog_product_name
from gluemaster.headfiles.tpl_cpt import Ui_Dialog_tpl_cpt
from gluemaster.headfiles.point_reset import Ui_Dialog_point_reset
from gluemaster.headfiles.parameter_reset import Ui_Dialog_parameter_reset
from gluemaster.headfiles.delete_accept import Ui_Dialog_delete_accept
import gluemaster.headfiles.gluemaster_config as gc
import rclpy, time, math, csv, os, shutil
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from example_interfaces.srv import Trigger, SetBool, AddTwoInts
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image
import warnings
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

warnings.filterwarnings("ignore", category=RuntimeWarning)

point_color = QColor(Qt.red)
line_color = QColor(Qt.black)
flag = 0
img_accurate = None
img_rough = None

class GUIClient_node(Node):
    def __init__(self, window):
        super().__init__("gui_client")
        # self.calibration_flag = 2
        self.window = window
        # Srv_group_ui = ReentrantCallbackGroup()
        Client_group_ui = MutuallyExclusiveCallbackGroup()
        self.client_start = self.create_client(Trigger, 'StartSrv',callback_group=Client_group_ui)
        self.client_stop = self.create_client(Trigger, 'ShutDownSrv',callback_group=Client_group_ui)
        self.client_reset = self.create_client(Trigger, 'ResetSrv',callback_group=Client_group_ui)
        self.client_rough = self.create_client(Trigger, 'Move2RecRoughPosSrv',callback_group=Client_group_ui)
        self.client_accurate = self.create_client(Trigger, 'Move2RecAccuratePosSrv',callback_group=Client_group_ui)
        self.client_calibration = self.create_client(Trigger, 'CalibrationModeSrv',callback_group=Client_group_ui)
        self.client_ChangeCam = self.create_client(AddTwoInts, 'Change_Using_cam',callback_group=Client_group_ui)
        self.client_test = self.create_client(AddTwoInts, 'TestRecSrv',callback_group=Client_group_ui)
        self.client_ruler = self.create_client(AddTwoInts, 'DemarcateSrv',callback_group=Client_group_ui)
        if not self.client_start.wait_for_service(timeout_sec=60): 
            raise RuntimeError('startservice not available') 
        if not self.client_stop.wait_for_service(timeout_sec=16):
            raise RuntimeError('stopservice not available') 
        if not self.client_reset.wait_for_service(timeout_sec=14): 
            raise RuntimeError('resetservice not available') 
        if not self.client_rough.wait_for_service(timeout_sec=14): 
            raise RuntimeError('roughservice not available') 
        if not self.client_accurate.wait_for_service(timeout_sec=14): 
            raise RuntimeError('accurateservice not available') 
        if not self.client_calibration.wait_for_service(timeout_sec=14): 
            raise RuntimeError('calibrationservice not available') 
        if not self.client_ChangeCam.wait_for_service(timeout_sec=14): 
            raise RuntimeError('ChangeCamservice not available')
        if not self.client_test.wait_for_service(timeout_sec=14): 
            raise RuntimeError('testservice not available')
        if not self.client_ruler.wait_for_service(timeout_sec=14): 
            raise RuntimeError('demarcateservice not available')

    def send_request(self, service_name, which=0):
        start_time = time.time()
        self.window.write_to_textEdit("         开始发送请求……")
        request = Trigger.Request()
        if service_name == 'start':
            future = self.client_start.call_async(request)
        elif service_name == 'stop':
            future = self.client_stop.call_async(request)
        elif service_name == 'reset':
            future = self.client_reset.call_async(request)
        elif service_name == 'move2roughpos':
            future = self.client_rough.call_async(request)
        elif service_name == 'move2accuratepos':
            future = self.client_accurate.call_async(request)
        elif service_name == 'calibration':
            future = self.client_calibration.call_async(request)
        elif service_name == 'changecam':
            request = AddTwoInts.Request()
            self.window.which_camera_is_editing = which
            request.a = which
            request.b = 1
            future = self.client_ChangeCam.call_async(request)
        elif service_name == 'test':
            request = AddTwoInts.Request()
            request.a = which
            future = self.client_test.call_async(request)
        elif service_name == 'ruler':
            request = AddTwoInts.Request()
            request.a = self.window.get_chess_board_side_length()
            request.b = which
            future = self.client_ruler.call_async(request)
        self.window.write_to_textEdit("         等待系统回应……")
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if hasattr(response,'success'):
            self.get_logger().info(f'响应结果:')
        elif hasattr(response,'sum'):
            self.get_logger().info(f'响应结果:')
        else:
            self.get_logger().error('服务调用失败')
        self.window.write_to_textEdit("         收到系统回应，开始工作！")
        print(time.time()-start_time)

class UI_Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        Subscriber_group_ui_calibration = MutuallyExclusiveCallbackGroup()
        Subscriber_group_ui_RE = MutuallyExclusiveCallbackGroup()
        self.speed = 0
        self.calibration_flag = 3
        self.encoder_subscription = self.create_subscription(
                Float32,'belt_speed_topic',self.encoder_listener_callback,callback_group=Subscriber_group_ui_RE,qos_profile=gc.qos_profile)
        self.calibration_subscription = self.create_subscription(
                Int32,'CalibrationFlag',self.calibration_listener_callback,callback_group=Subscriber_group_ui_calibration,qos_profile=gc.qos_profile)

    def encoder_listener_callback(self, msg):
        self.speed = msg.data
        
    def calibration_listener_callback(self, msg):
        self.calibration_flag = msg.data
        print("self.calibration_flag = ", self.calibration_flag)

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        Subscriber_group_ui_Cam = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
                Image,'Original_Image',self.image_callback,callback_group=Subscriber_group_ui_Cam,qos_profile=gc.qos_profile)
        self.bridge = CvBridge()
        self.current_frame = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f'图像转换错误: {e}')

class Ros2Thread(QThread):
    update_signal = pyqtSignal()
    def __init__(self, subscriber_node):
        super().__init__()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(subscriber_node)

    def run(self):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            self.update_signal.emit()

class Dialog_painter_set(QDialog, Ui_Dialog_painter_set, Ui_MainWindow):
    def __init__(self,window):
        super().__init__()
        self.setupUi(self)
        self.window=window
    
        # 初始化绘图颜色设置
        self.button_PointColor.setStyleSheet(f"background-color: {point_color.name()};")
        self.button_LineColor.setStyleSheet(f"background-color: {line_color.name()};")

        self.button_PointColor.clicked.connect(self.choose_point_color)
        self.button_LineColor.clicked.connect(self.choose_line_color)
        self.buttonBox.accepted.connect(self.on_accept)
        self.buttonBox.rejected.connect(self.on_reject)

    def on_accept(self):
        global flag
        flag = 1

    def on_reject(self):
        row_count = self.window.tableWidget.rowCount()
        self.window.tableWidget.removeRow(row_count - 1)
        
    def choose_point_color(self):
        global point_color
        color = QColorDialog.getColor(point_color, self, "选择点颜色")
        if color.isValid():
            point_color = color
            self.button_PointColor.setStyleSheet(f"background-color: {color.name()};")

    def choose_line_color(self):
        global line_color
        color = QColorDialog.getColor(line_color, self, "选择线颜色")
        if color.isValid():
            line_color = color
            self.button_LineColor.setStyleSheet(f"background-color: {color.name()};")

    def get_selected_option(self):
        if self.comboBox.currentIndex() == 0:
            return 1
        elif self.comboBox.currentIndex() == 1:
            return 2
        elif self.comboBox.currentIndex() == 2:
            return 3
        else:
            return None

class Dialog_mv_set(QDialog, Ui_Dialog_mv_set, Ui_MainWindow):
    def __init__(self,window):
        super().__init__()
        self.setupUi(self)
        self.window=window

        self.doubleSpinBox_time.setRange(0.0, 100.0)    # 设置最小值和最大值
        self.doubleSpinBox_time.setSingleStep(0.1)      # 设置步长为0.1
        self.doubleSpinBox_time.setValue(1.0)           # 设置初始值为1.0
        self.doubleSpinBox_time.setDecimals(3)          # 设置显示的小数位数为3位
        self.doubleSpinBox_height.setRange(0.0, 100.0)  # 设置最小值和最大值
        self.doubleSpinBox_height.setSingleStep(0.1)    # 设置步长为0.1
        self.doubleSpinBox_height.setValue(1.0)         # 设置初始值为1.0
        self.doubleSpinBox_height.setDecimals(3)        # 设置显示的小数位数为3位
        self.buttonBox.accepted.connect(self.on_accept)
        self.buttonBox.rejected.connect(self.on_reject)

    def get_num_value(self):
        return self.doubleSpinBox_time.value(), self.doubleSpinBox_height.value()
    
    def on_accept(self):
        pass

    def on_reject(self):
        row_count = self.window.tableWidget.rowCount()
        self.window.tableWidget.removeRow(row_count - 1)

class Dialog_product_name(QDialog, Ui_Dialog_product_name, Ui_MainWindow):
    def __init__(self, window, type="create"):
        super().__init__()
        self.setupUi(self)
        self.window=window
        self.type = type
        self.btn_accept.clicked.connect(self.on_accept)
        self.btn_reject.clicked.connect(self.on_reject)
        self.label_info.setText("注意：输入不能为空且不能重复！")
        self.state = ''
        
    def get_text(self):
        return self.lineEdit.text()
    
    def on_accept(self):
        input_text = self.lineEdit.text()
        self.state = 'accept'
        if not input_text:
            self.label_info.setStyleSheet("color: red;")
            self.label_info.setText("错误：输入不能为空！")
            self.lineEdit.setFocus()
            return
        if input_text in self.window.existing_name:
            self.label_info.setStyleSheet("color: red;")
            self.label_info.setText(f"错误：'{input_text}'已存在！")
            self.lineEdit.selectAll()
            self.lineEdit.setFocus()
            return
        self.accept()

    def on_reject(self):
        if self.type=="create":
            self.state = 'cancel'
            # row_count = self.window.window.tableWidget_product.rowCount()
            # self.window.window.tableWidget_product.removeRow(row_count - 1)
        elif self.type=="edit":
            self.state = 'cancel'
        self.accept()

class Dialog_point_reset(QDialog, Ui_Dialog_point_reset, Ui_MainWindow):
    def __init__(self, window):
        super().__init__()
        self.setupUi(self)
        self.window = window
        self.state = ''
        self.btn_accept.clicked.connect(self.on_accept)
        self.btn_reject.clicked.connect(self.on_reject)
        self.label_info.setText(f"提示: x范围:(0, {round(self.window.data_interface.pixel2real(self.window.template_width, 'x'),3)}),y范围(0, {round(self.window.data_interface.pixel2real(self.window.template_height, 'y'),3)})")
        validator1 = QDoubleValidator()
        validator1.setRange(0, 999, 3)
        self.lineEdit_x.setValidator(validator1)
        validator2 = QDoubleValidator()
        validator2.setRange(0, 999, 3)
        self.lineEdit_y.setValidator(validator2)
        
    def get_text(self):
        return self.lineEdit_x.text(), self.lineEdit_y.text()
    
    def error_text_show(self, text):
        self.label_info.setStyleSheet("color: red;")
        self.label_info.setText(text)
        # self.lineEdit.setFocus()
    
    def on_accept(self):
        input_x, input_y = self.get_text()
        self.state = 'accept'
        if not input_x:
            self.error_text_show("错误: x坐标不能为空!")
            return
        elif not input_y:
            self.error_text_show("错误: y坐标不能为空!")
            return
        x, y = eval(input_x), eval(input_y)
        if x<0 or x>self.window.data_interface.pixel2real(self.window.template_width, 'x'):
            self.error_text_show("错误: x坐标超出模板范围!")
            return
        elif y<0 or y>self.window.data_interface.pixel2real(self.window.template_height, 'y'):
            self.error_text_show("错误: y坐标超出模板范围!")
            return
        self.accept()

    def on_reject(self):
        self.state = 'cancel'
        self.accept()

class Dialog_parameter_reset(QDialog, Ui_Dialog_parameter_reset, Ui_MainWindow):
    def __init__(self, window, type):
        super().__init__()
        self.setupUi(self)
        self.window = window
        self.type = type
        self.state = ''
        self.btn_accept.clicked.connect(self.on_accept)
        self.btn_reject.clicked.connect(self.on_reject)

        validator = QDoubleValidator()
        validator.setRange(0, 100, 2)
        self.lineEdit.setValidator(validator)

        if self.type == 'time':
            self.label.setText("点胶时间：")
            self.label_info.setText("提示: 单位为秒(s)")
        elif self.type == 'height':
            self.label.setText("点胶高度：")
            self.label_info.setText("提示: 单位为毫米(mm)")
        
    def get_text(self):
        return self.lineEdit.text()
    
    def error_text_show(self, text):
        self.label_info.setStyleSheet("color: red;")
        self.label_info.setText(text)
        # self.lineEdit.setFocus()
    
    def on_accept(self):
        input = self.get_text()
        self.state = 'accept'
        if not input:
            self.error_text_show("错误: 输入不能为空!")
            return
        pmt = eval(input)
        if pmt<=0 or pmt>200:
            self.error_text_show("错误: 输入数据不合理!")
            return
        self.accept()

    def on_reject(self):
        self.state = 'cancel'
        self.accept()

class Dialog_delete_accept(QDialog, Ui_Dialog_delete_accept, Ui_MainWindow):
    def __init__(self, window):
        super().__init__()
        self.setupUi(self)
        self.window=window
        self.buttonBox.accepted.connect(self.on_accept)
        self.buttonBox.rejected.connect(self.on_reject)
        self.label_tpl.setText(f"{self.window.window.tableWidget_product.item(self.window.window.selected_product,0).text()}")

    
    def on_accept(self):
        pass

    def on_reject(self):
        pass

class Dialog_tpl_cpt(QDialog, Ui_Dialog_tpl_cpt, Ui_MainWindow):
    def __init__(self, window, flag):
        super().__init__()
        self.setupUi(self)
        self.window = window
        self.btnCapture.clicked.connect(self.capture_img)
        self.btnCutTpl.clicked.connect(self.cut_template)
        self.btnSaveTpl.clicked.connect(self.save_template)

        self.scene = QGraphicsScene(self)
        self.graphicsView.setScene(self.scene)
        self.pixmap_item = QGraphicsPixmapItem()
        self.scene.addItem(self.pixmap_item)
        
        self.flag = flag
        self.freeze_frame = False
        self.start_point = None
        self.end_point = None
        self.cornor_selected = None
        self.angle = 0
        self.is_drawing = False
        self.drawonepoint = False
        self.is_square_finalized = False
        self.selected = False
        self.dragging = False
        self.current_polygon_item = None
        self.x = [None, None]
        self.y = [None, None]
        self.cornor_angle = [None, None, None, None]
        self.R = None
        self.x0 = None
        self.y0 = None
        self.cx = None
        self.cy = None
        self.square_part_item = []
        self.move_step = 1
        self.rotate_step = 2
        self.scale = None

        self.camera_subscriber_capture = CameraSubscriber()
        self.ros2_thread_capture = Ros2Thread(self.camera_subscriber_capture)
        self.ros2_thread_capture.update_signal.connect(self.update_video_capture)
        self.ros2_thread_capture.start()

        self.graphicsView.setMouseTracking(True)
        self.graphicsView.viewport().installEventFilter(self)

        if self.flag:
            self.label.setText("当前为精模板处理！摆好产品后请点击“拍摄模板”按钮！")
        else:
            self.label.setText("当前为粗模板处理！摆好产品后请点击“拍摄模板”按钮！")

    def update_video_capture(self):
        if self.camera_subscriber_capture.current_frame is not None and self.freeze_frame == 0:
            self.frame = self.camera_subscriber_capture.current_frame
            h, w, ch = self.frame.shape
            bytes_per_line = ch * w
            q_img = QImage(self.frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            original_size = QSize(w, h)
            pixmap = QPixmap.fromImage(q_img).scaled(self.graphicsView.size(), Qt.KeepAspectRatio)
            scaled_size = pixmap.size()
            self.pixmap_item.setPixmap(pixmap)
            self.graphicsView.fitInView(self.pixmap_item, Qt.KeepAspectRatio)
            self.scale = scaled_size.width() / original_size.width()
        else:
            time.sleep(0.001)

    # def resizeEvent(self, event):
    #     """窗口大小改变时重新调整视图"""
    #     if not self.pixmap_item.pixmap().isNull():
    #         self.graphicsView.fitInView(self.pixmap_item, Qt.KeepAspectRatio)
    #     super().resizeEvent(event)

    def capture_img(self):
        if self.freeze_frame == False:
            self.freeze_frame = True
            if self.flag:
                self.label.setText("当前为精模板处理！请点击“截取模板”按钮截取模板！")
            else:
                self.label.setText("当前为粗模板处理！请点击“截取模板”按钮截取模板！")

    def cut_template(self):
        if self.freeze_frame == True:
            self.startDrawing()
            if self.flag:
                self.label.setText("当前为精模板处理！请点击“保存模板”按钮保存截取的模板！")
            else:
                self.label.setText("当前为粗模板处理！请点击“保存模板”按钮保存截取的模板！")

    def save_template(self):
        self.frame=cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
        self.x[0] = int(self.x[0] / self.scale)
        self.x[1] = int(self.x[1] / self.scale)
        self.y[0] = int(self.y[0] / self.scale)
        self.y[1] = int(self.y[1] / self.scale)
        cropped = self.frame[self.y[0]:self.y[1], self.x[0]:self.x[1]]
        # cv2.imshow("d", self.frame)
        if self.flag:
            self.window.product_accurate_image_path_temp = "./gluemaster_files/temp/accurate.jpg"
            cv2.imwrite(self.window.product_accurate_image_path_temp, cropped)
        else:
            self.window.product_rough_image_path_temp = "./gluemaster_files/temp/rough.jpg"
            cv2.imwrite(self.window.product_rough_image_path_temp, cropped)
        self.window.data_interface.cap_statue_flag += 1
        self.close()
    
    def startDrawing(self):
        self.is_drawing = True
        self.start_point = None
        self.end_point = None
        self.cornor_selected = None
        self.drawonepoint = False
        self.selected = False
        self.angle = 0
        self.is_square_finalized = False
        self.dragging = False
        self.del_paint_square()
    
    def stopDrawing(self):
        self.is_drawing = False
        self.start_point = None
        self.end_point = None
        self.cornor_selected = None
        self.drawonepoint = False
        self.selected = False
        self.angle = 0
        self.is_square_finalized = False
        self.dragging = False
        self.del_paint_square()

    def keyPressEvent(self, event):
        global flag
        if event.key() == Qt.Key_Escape:
            self.scene.removeItem(self.temp)
            self.del_paint_square()
            self.stopDrawing()

        if event.key() == Qt.Key_A and self.selected:
            self.square_move(-self.move_step, 0, 0)
        elif event.key() == Qt.Key_D and self.selected:
            self.square_move(self.move_step, 0, 0)
        elif event.key() == Qt.Key_W and self.selected:
            self.square_move(0, -self.move_step, 0)
        elif event.key() == Qt.Key_S and self.selected:
            self.square_move(0, self.move_step, 0)

        if event.key() == Qt.Key_V and self.selected:
            self.square_rotate(self.rotate_step)
        if event.key() == Qt.Key_Z and self.selected:
            self.square_rotate(-self.rotate_step)

        else:
            super().keyPressEvent(event)

    def eventFilter(self, source, event):
        """处理鼠标事件"""
        global point_color, line_color, flag
        if source is self.graphicsView.viewport():
            # 鼠标移动事件处理
            if event.type() == QtCore.QEvent.MouseMove:
                pos = self.graphicsView.mapToScene(event.pos())
                x, y = int(pos.x()), int(pos.y())
                    
                if self.drawonepoint:
                    self.paint_square(self.x[0], self.y[0], x, y, 1, 0)
                if self.is_square_finalized and self.dragging:
                    self.square_move(x, y)
                if self.cornor_selected:
                    if self.cornor_selected == 1:
                        self.x[0], self.y[0] = x, y
                    elif self.cornor_selected == 2:
                        self.x[1], self.y[0] = x, y
                    elif self.cornor_selected == 3:
                        self.x[1], self.y[1] = x, y
                    elif self.cornor_selected == 4:
                        self.x[0], self.y[1] = x, y
                    self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], 1)

            # 鼠标释放事件处理
            elif event.type() == QtCore.QEvent.MouseButtonRelease and self.is_square_finalized:
                time.sleep(0.01)
                if event.type() == QtCore.QEvent.MouseButtonRelease and self.is_square_finalized:
                    self.dragging = False
                    self.cornor_selected = None
                    self.cornor_update()

            # 鼠标点击事件处理
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    pos = self.graphicsView.mapToScene(event.pos())
                    x, y = int(pos.x()), int(pos.y())

                    if not (0 <= x <= 1200 and 0 <= y <= 900):
                        return True
                    
                    if self.x[0] and self.x[1] and self.is_square_finalized:
                        w = 20
                        if self.current_polygon_item.contains(QPointF(x, y)):
                            self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], 2)
                            self.cornor_selected = None
                            if self.x[0]-w<=x<=self.x[0]+w and self.y[0]-w<=y<=self.y[0]+w:
                                self.cornor_selected = 1
                            elif self.x[1]-w<=x<=self.x[1]+w and self.y[0]-w<=y<=self.y[0]+w:
                                self.cornor_selected = 2
                            elif self.x[1]-w<=x<=self.x[1]+w and self.y[1]-w<=y<=self.y[1]+w:
                                self.cornor_selected = 3
                            elif self.x[0]-w<=x<=self.x[0]+w and self.y[1]-w<=y<=self.y[1]+w:
                                self.cornor_selected = 4
                            else:
                                self.dragging = True
                                self.selected = True
                                self.x0, self.y0 = x, y
                        else:
                            self.selected = False
                            self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], 1)
            
                    # 正四边形模式
                    if self.is_drawing:
                        if not self.start_point:
                            self.start_point = (x, y)
                            self.x[0], self.y[0] = x, y
                            self.temp = self.scene.addEllipse(self.x[0]-2, self.y[0]-2, 4, 4,QPen(point_color),QBrush(point_color))
                            self.drawonepoint = True
                        elif not self.end_point:
                            self.end_point = (x, y)
                            self.x[1], self.y[1] = x, y
                            self.cx = (self.x[0]+self.x[1])/2
                            self.cy = (self.y[0]+self.y[1])/2
                            self.R = math.sqrt((self.cx-self.x[0])**2+(self.cy-self.y[0])**2)
                            self.is_square_finalized = True
                            self.is_drawing = False
                            self.drawonepoint = False
                            self.scene.removeItem(self.temp)
                            self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], 1)
                    elif self.is_square_finalized:
                        self.cornor_update()
                
        return super().eventFilter(source, event)

    def paint_square(self, x1, y1, x2, y2, bold, type=1):
        self.del_paint_square()
        x0 = (x1 + x2)/2
        y0 = (y1 + y2)/2
        polygon = QPolygonF([
            QPointF(x1, y1),QPointF(x2, y1),
            QPointF(x2, y2),QPointF(x1, y2)
        ])
        poly_item = QGraphicsPolygonItem(polygon)
        self.current_polygon_item = poly_item
        if type == 0:
            poly_item.setPen(QPen(line_color, 2*bold, Qt.DashLine))
        elif type == 1:
            poly_item.setPen(QPen(line_color, 2*bold))
        poly_item.setBrush(QBrush(QColor(200, 200, 255, 30))) # 半透明蓝色
        self.scene.addItem(poly_item)
        self.square_part_item.append(poly_item)
        self.point_ul = self.scene.addEllipse(x1-2*bold, y1-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
        self.point_ur = self.scene.addEllipse(x1-2*bold, y2-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
        self.point_dr = self.scene.addEllipse(x2-2*bold, y2-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
        self.point_dl = self.scene.addEllipse(x2-2*bold, y1-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
        self.point_ct = self.scene.addEllipse(x0-2*bold, y0-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
        line1 = QGraphicsLineItem(x1, y1, x2, y2)
        line2 = QGraphicsLineItem(x1, y2, x2, y1)
        line1.setPen(QPen(line_color, 1*bold, Qt.DashLine))
        line2.setPen(QPen(line_color, 1*bold, Qt.DashLine))
        self.scene.addItem(line1)
        self.scene.addItem(line2)
        self.square_part_item += [line1, line2, self.point_ul, self.point_ur, self.point_dr, self.point_dl, self.point_ct]

    def del_paint_square(self):
        for i in range(len(self.square_part_item)):
            self.scene.removeItem(self.square_part_item[i])
        self.square_part_item = []

    def square_move(self, x, y, mode=1):
        if mode:
            dx = x - self.x0
            dy = y - self.y0
            self.x0, self.y0 = x, y
        else:
            dx, dy = x, y
        for i in range(2):
            self.x[i] += dx
            self.y[i] += dy
        self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], 2)
        self.cornor_update()

    def cornor_update(self):
        self.cx = (self.x[0]+self.x[1])/2
        self.cy = (self.y[0]+self.y[1])/2
        self.R = math.sqrt((self.cx-self.x[0])**2+(self.cy-self.y[0])**2)

class Data_Interface(Ui_MainWindow):
    def __init__(self,window):
        super().__init__()
        self.window=window
        self.existing_name = []
        self.img_choose = None
        self.is_adding_product = False
        self.is_editting_product = False
        self.name_edited = False
        self.old_name = ''
        self.new_name = ''
        self.box = []
        self.name_state = ''
        self.cap_statue_flag = 0
        self.tpl_scale = None
        self.ruler_x = None
        self.ruler_y = None
        self.folder_path =[
            "./gluemaster_files/CSVMethods",
            "./gluemaster_files/temp",
            "./gluemaster_files/tpl_accurate",
            "./gluemaster_files/tpl_rough",
        ] 
        self.get_ruler()

    def get_ruler(self):
        rulerdata = np.load("./gluemaster_files/rulerdata.npz")
        ruler = rulerdata['ruler_accurate']
        self.ruler_x, self.ruler_y = ruler[0], ruler[1]
        print("ruler", self.ruler_x, self.ruler_y)

    def pixel2real(self, pix, which):
        if which == 'x':
            ret = (pix/self.tpl_scale)/self.ruler_x
        elif which == 'y':
            ret = (pix/self.tpl_scale)/self.ruler_y
        return ret

    def real2pixel(self, real, which):
        if which == 'x':
            ret = (real*self.tpl_scale)*self.ruler_x
        elif which == 'y':
            ret = (real*self.tpl_scale)*self.ruler_y
        return ret
    
    def tuple_pixel2real(self, t):
        temp_list = list(t)
        temp_list[0] = self.pixel2real(temp_list[0], 'x')
        temp_list[1] = self.pixel2real(temp_list[1], 'y')
        return tuple(temp_list)
    
    def tuple_real2pixel(self, t):
        temp_list = list(t)
        temp_list[0] = self.real2pixel(temp_list[0], 'x')
        temp_list[1] = self.real2pixel(temp_list[1], 'y')
        return tuple(temp_list)
    
    def add_product(self):
        self.window.write_to_textEdit_2("         正在添加新的模板！")
        self.window.write_to_textEdit_2("         正在添加新的模板！")
        if self.cap_statue_flag == 0:
            self.window.guiclient_node.send_request('move2roughpos')
            self.window.guiclient_node.send_request('changecam', 0)
            dialog5 = Dialog_tpl_cpt(self.window, 0)
            dialog5.exec_()
        if self.cap_statue_flag == 1:
            self.window.guiclient_node.send_request('changecam', 1)
            self.window.guiclient_node.send_request('move2accuratepos')
            dialog5 = Dialog_tpl_cpt(self.window, 1)
            dialog5.exec_()
            self.window.guiclient_node.send_request('changecam', 0)
            self.window.guiclient_node.send_request('move2roughpos', 0)
        if self.cap_statue_flag == 2:
            self.set_product_name()
        if self.cap_statue_flag == 3:
            row_position = self.window.tableWidget_product.rowCount()  # 获取当前行数
            self.window.tableWidget_product.insertRow(row_position)  # 插入新行
            self.window.product_EN = 1
            self.window.write_to_product_cells(row_position,0,QTableWidgetItem(self.window.product_name))
            self.window.write_to_product_cells(row_position,1,1)
            self.cap_statue_flag = 0
            self.is_adding_product = True
            self.import_template_image(1,2)
        if self.is_adding_product:
            self.window.btnAddProduct.setEnabled(False)
            self.window.btnExport.setEnabled(True)
            self.window.btnDeleteProduct.setEnabled(False)
            self.window.btnAdd.setEnabled(True)
            self.window.btnDelete.setEnabled(True)
            self.window.btnEdit.setEnabled(False)
            self.window.btnRestore.setEnabled(False)
            self.window.btnSave.setEnabled(False)
        else:
            self.cap_statue_flag = 0
            self.window.write_to_textEdit_2("         已取消添加新模板！")
        
    def set_product_name(self, type="create"):
        self.read_all_product_name()
        dialog4 = Dialog_product_name(window=self, type=type)
        dialog4.exec_()
        self.name_state = dialog4.state
        if dialog4.state == 'accept':
            dialog4.state = ''
            self.window.product_name = dialog4.get_text()
            self.cap_statue_flag += 1

    def read_all_product_name(self):
        self.existing_name = []
        for row in range(self.window.tableWidget_product.rowCount()):
            item = self.window.tableWidget_product.item(row, 0)
            self.existing_name.append(item.text() if item is not None else "")

    def export_product(self):
        if self.is_adding_product:
            self.is_adding_product = False
            self.window.btnAddProduct.setEnabled(True)
            self.window.btnExport.setEnabled(False)
            self.window.btnDeleteProduct.setEnabled(True)
            self.window.btnAdd.setEnabled(False)
            self.window.btnDelete.setEnabled(False)
            self.window.btnEdit.setEnabled(True)
            self.window.btnRestore.setEnabled(False)
            self.window.btnSave.setEnabled(False)
            self.export_to_images()
            self.export_csv()
            self.write_to_connections()
            self.save_product()
            self.read_all_product_name()
            self.window.read_gluemaster_files()
            self.window.write_to_textEdit_2("         新模板导出并保存成功！")

    def edit_product(self):
        if self.window.selected_product is not None:
            self.window.write_to_textEdit_2("         正在编辑模板！")
            self.is_editting_product=True
            self.box[self.window.selected_product].view().setEnabled(True)

            self.window.btnAddProduct.setEnabled(False)
            self.window.btnExport.setEnabled(False)
            self.window.btnDeleteProduct.setEnabled(False)
            self.window.btnAdd.setEnabled(True)
            self.window.btnDelete.setEnabled(True)
            self.window.btnEdit.setEnabled(False)
            self.window.btnRestore.setEnabled(True)
            self.window.btnSave.setEnabled(True)
            self.import_template_image(0,0)
            self.import_csv(0)
            # print("w,h:", self.window.template_width, self.window.template_height)

    def restore_product(self):
        if self.is_editting_product:
            self.import_csv(-1)
            self.restore_table_product()
            self.is_editting_product=False
            self.window.btnAddProduct.setEnabled(True)
            self.window.btnExport.setEnabled(False)
            self.window.btnDeleteProduct.setEnabled(True)
            self.window.btnAdd.setEnabled(False)
            self.window.btnDelete.setEnabled(False)
            self.window.btnEdit.setEnabled(True)
            self.window.btnRestore.setEnabled(False)
            self.window.btnSave.setEnabled(False)
            self.window.write_to_textEdit_2("         已重置当前模板及其动作！")
        
    def restore_table_product(self):
        with open("./gluemaster_files/connections.csv", 'r', newline='', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            rowdata = list(reader)
            name = rowdata[self.window.selected_product+1][0]
            en = rowdata[self.window.selected_product+1][1]
            print(name, en)
        self.window.read_gluemaster_files()
        # self.box[self.window.selected_product].view().setEnabled(True)
        for i in range(len(self.box)):
            self.box[i].view().setEnabled(False)
    
    def change_all_name(self):
        old_tpl_accurate_name = "./gluemaster_files/tpl_accurate/"+self.old_name+".jpg"
        new_tpl_accurate_name = "./gluemaster_files/tpl_accurate/"+self.new_name+".jpg"
        os.rename(old_tpl_accurate_name, new_tpl_accurate_name)
        old_tpl_rough_name = "./gluemaster_files/tpl_rough/"+self.old_name+".jpg"
        new_tpl_rough_name = "./gluemaster_files/tpl_rough/"+self.new_name+".jpg"
        os.rename(old_tpl_rough_name, new_tpl_rough_name)
        old_CSVMethods_name = "./gluemaster_files/CSVMethods/"+self.old_name+".csv"
        new_CSVMethods_name = "./gluemaster_files/CSVMethods/"+self.new_name+".csv"
        os.rename(old_CSVMethods_name, new_CSVMethods_name)
        print(f"文件已从 {self.old_name} 改名为 {self.new_name}")
        self.window.product_rough_image_path = new_tpl_rough_name
        self.window.product_accurate_image_path = new_tpl_accurate_name
        self.window.product_method_path = new_CSVMethods_name

    def save_product(self):
        if self.is_editting_product:
            self.is_editting_product=False
            if self.name_edited:
                self.change_all_name()
                self.name_edited = False

            self.window.btnAddProduct.setEnabled(True)
            self.window.btnExport.setEnabled(False)
            self.window.btnDeleteProduct.setEnabled(True)
            self.window.btnAdd.setEnabled(False)
            self.window.btnDelete.setEnabled(False)
            self.window.btnEdit.setEnabled(True)
            self.window.btnRestore.setEnabled(False)
            self.window.btnSave.setEnabled(False)
            self.window.product_name = self.window.tableWidget_product.item(self.window.selected_product,0).text()
            self.window.product_EN = int(self.box[self.window.selected_product].currentData())
            
            with open(self.window.product_connection_path, 'r', newline='') as f:
                reader = csv.reader(f)
                data = list(reader)
                data[self.window.selected_product+1][0] = self.window.product_EN
                data[self.window.selected_product+1][1] = self.window.product_name
                data[self.window.selected_product+1][2] = self.window.product_rough_image_path
                data[self.window.selected_product+1][3] = self.window.product_accurate_image_path
                data[self.window.selected_product+1][4] = self.window.product_method_path
            with open(self.window.product_connection_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(data)
            self.export_csv()
            self.window.read_gluemaster_files()
            self.window.write_to_textEdit_2("         模板编辑部分已保存成功！")

    def delete_product(self):
        # print(self.window.selected_product)
        if self.window.selected_product is not None:
            dialog = Dialog_delete_accept(window=self)
            if dialog.exec_() == QDialog.Accepted and self.window.selected_product<self.window.tableWidget_product.rowCount():
                self.window.clear_scene()
                with open(self.window.product_connection_path, 'r', newline='') as f:
                    reader = csv.reader(f)
                    rowdata = list(reader)
                    self.window.product_image_path = rowdata[self.window.selected_product+1][2]
                    self.window.product_method_path = rowdata[self.window.selected_product+1][3]
                    rowdata.pop(self.window.selected_product+1)
                with open(self.window.product_connection_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(rowdata)
                    os.remove(self.window.product_image_path)
                    os.remove(self.window.product_method_path)
                self.window.tableWidget.setRowCount(0)
                self.window.read_gluemaster_files()
                self.window.clear_scene()
                if self.window.background_item:
                    self.window.scene.removeItem(self.window.background_item)
                self.window.write_to_textEdit_2("         已删除当前模板！")
            else:
                self.window.write_to_textEdit_2("         已取消删除模板！")
        else:
            self.window.write_to_textEdit_2("         请先选中模板！")

    def import_template_image(self, img_type, type):
        if type==1:
            if img_type == 1:
                rgb_image = cv2.cvtColor(img_accurate, cv2.COLOR_BGR2RGB)
            else :
                rgb_image = cv2.cvtColor(img_rough, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            q_img = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(q_img)
        elif type==0:
            with open(self.window.product_connection_path, 'r', encoding='utf-8') as csv_file:
                reader = csv.reader(csv_file)
                rowdata = list(reader)
                self.window.product_EN = rowdata[self.window.selected_product+1][0]
                self.window.product_name = rowdata[self.window.selected_product+1][1]
                self.window.product_rough_image_path = rowdata[self.window.selected_product+1][2]
                self.window.product_accurate_image_path = rowdata[self.window.selected_product+1][3]
                self.window.product_method_path = rowdata[self.window.selected_product+1][4]
                self.template_path = self.window.product_accurate_image_path
                pixmap = QPixmap(self.template_path)
        elif type==2:
            self.template_path = self.window.product_accurate_image_path_temp
            pixmap = QPixmap(self.template_path)

        self.window.tableWidget.setRowCount(0)
        if self.window.background_item:
            self.window.scene.removeItem(self.window.background_item)
        self.window.point_line_item = []
        self.window.points_pos = []
        # 加载并处理图片
        # pixmap = QPixmap(self.template_path)
        if not pixmap.isNull():
            # 计算保持比例的缩放
            scaled_pix = pixmap.scaled(self.window.scene_width,self.window.scene_height,aspectRatioMode=Qt.KeepAspectRatio,transformMode=Qt.SmoothTransformation)
            self.tpl_scale = scaled_pix.width() / pixmap.width()
            print("tpl_scale", self.tpl_scale, scaled_pix.width(), pixmap.width())
            # 创建图形项并定位
            self.window.background_item = QGraphicsPixmapItem(scaled_pix)
            self.window.background_item.setPos(0, 0)  # 显式设置左上角坐标
            # 设置层级和添加
            self.window.background_item.setZValue(-1)
            self.window.scene.addItem(self.window.background_item)
            # 调整视图确保可见区域
            self.window.graphicsView.ensureVisible(0, 0, 10, 10)
            bottomRight = self.window.background_item.boundingRect().bottomRight()
            self.window.template_width = bottomRight.x()
            self.window.template_height = bottomRight.y()
            self.window.last_x = self.window.template_width/2
            self.window.last_y = self.window.template_height/2
            self.window.clear_scene()

    def export_to_images(self):
        target_folder_accurate = "./gluemaster_files/tpl_accurate"  # 目标文件夹名称
        target_folder_rough = "./gluemaster_files/tpl_rough"
        target_file_name =  self.window.product_name + '.jpg' # 自定义目标文件名
        self.window.product_accurate_image_path = os.path.join(target_folder_accurate, target_file_name)
        self.window.product_rough_image_path = os.path.join(target_folder_rough, target_file_name)
        shutil.copy(self.window.product_accurate_image_path_temp, self.window.product_accurate_image_path)  # 复制文件
        shutil.copy(self.window.product_rough_image_path_temp, self.window.product_rough_image_path)  # 复制文件


    def write_to_connections(self):
        new_row = [self.window.product_EN, self.window.product_name, self.window.product_rough_image_path,
                   self.window.product_accurate_image_path, self.window.product_method_path]  # 新行数据
        with open(self.window.product_connection_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(new_row)  # 追加单行

    def export_csv(self):
        self.window.product_method_path = "./gluemaster_files/CSVMethods/"+self.window.product_name+".csv"  # 指定固定路径
        rows = self.window.tableWidget.rowCount()
        export_cols = [0, 3, 4, 5]  # 指定要导出的列
        with open(self.window.product_method_path, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            for row in range(rows):
                row_data = []
                for col in export_cols:
                    item = self.window.tableWidget.item(row, col)
                    row_data.append(item.text())
                if row_data[0] == 'movement':
                    row_data.append(0)
                else:
                    row_data.append(1)
                writer.writerow(row_data)

    def import_csv(self, flag=1):
        if flag==1:
            self.csv_path, _ = QFileDialog.getOpenFileName(
                parent=self.window.tableWidget,
                caption="打开CSV文件",
                directory="",
                filter="CSV Files (*.csv);;All Files (*)")
        elif flag==0:
            self.csv_path = self.window.product_method_path
        if self.csv_path:
            self.window.clear_scene()
            with open(self.csv_path, 'r', encoding='utf-8') as csv_file:
                reader = csv.reader(csv_file)
                self.rowdata = list(reader)
                self.row = len(self.rowdata)
                if self.rowdata:
                    self.window.tableWidget.clearContents()
                    self.window.tableWidget.setRowCount(0)
                    for row_data in self.rowdata:
                        current_row = self.window.tableWidget.rowCount()
                        self.window.tableWidget.insertRow(current_row)
                        for i in range(self.row):
                            self.window.write_to_item_cells(current_row, 0, QTableWidgetItem(str(row_data[0])))
                            self.window.write_to_item_cells(current_row, 3, QTableWidgetItem(str(row_data[1])))
                            self.window.write_to_item_cells(current_row, 4, QTableWidgetItem(str(row_data[2])))
                            self.window.write_to_item_cells(current_row, 5, QTableWidgetItem(str(row_data[3])))
                            # self.window.write_to_item_cells(current_row, 6, QTableWidgetItem(str(row_data[4])))
            self.revert_settings()

    def revert_settings(self):
        self.window.template_width
        self.window.template_height
        self.window.last_x = self.window.template_width/2
        self.window.last_y = self.window.template_height/2
        self.window.points_pos = []
        for i in range(self.row):
            self.calculate_points(i, self.rowdata[i])
        # print(self.window.points_pos)
        for i in range(self.row):
            first_point = self.window.round_tuple(self.tuple_pixel2real(self.window.points_pos[i][0]))
            self.window.write_to_item_cells(i, 1, QTableWidgetItem(str(first_point)))
            if self.rowdata[i][0] == 'circle':
                second_point = self.window.round_tuple(self.tuple_pixel2real(self.window.points_pos[i][2]))
            else:
                second_point = self.window.round_tuple(self.tuple_pixel2real(self.window.points_pos[i][1]))
            self.window.write_to_item_cells(i, 2, QTableWidgetItem(str(second_point)))
        self.window.point_line_item = []
        self.window.delete_all_item()
        self.window.draw_all_item()
        
    def calculate_points(self, row, datas):
        type = datas[0]
        displacement = eval(datas[1])
        if displacement[0] is not None:
            self.window.round_tuple(displacement)
        displacement = self.tuple_real2pixel(displacement)
        EN = eval(datas[4])
        self.window.points_pos.append([(self.window.last_x, self.window.last_y)])
        if type == 'point' or type == 'circle':
            x = self.window.last_x
            y = self.window.last_y
            self.window.points_pos[row].append((x, y))
            if type == 'circle':
                cx = self.window.last_x + displacement[0]
                cy = self.window.last_y + displacement[1]
                self.window.points_pos[row].append((cx, cy))
        elif type == 'line' or type == 'movement':
            x = self.window.last_x + displacement[0]
            y = self.window.last_y + displacement[1]
            self.window.points_pos[row].append((x, y))
        self.window.last_x, self.window.last_y = x, y 

    def clear_all_tpl(self):
        try:
            for i in range(4):
                shutil.rmtree(self.folder_path[i])  # 删除整个文件夹
                os.makedirs(self.folder_path[i])    # 重新创建空文件夹
            with open(self.window.product_connection_path, 'r', newline='') as f:
                    reader = csv.reader(f)
                    rowdata = list(reader)
                    rowdata[1:] = []
            with open(self.window.product_connection_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(rowdata)
            self.window.read_gluemaster_files()
        except Exception as e:
            print(f'清空模板失败. 原因: {e}')

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        self.data_interface = Data_Interface(window=self)

        rclpy.init()
        self.guiclient_node = GUIClient_node(window=self)
        self.ui_subscriber_node = UI_Subscriber()

        self.ros2_thread = Ros2Thread(self.ui_subscriber_node)
        self.ros2_thread.update_signal.connect(self.update_label)
        self.ros2_thread.start()

        self.camera_subscriber = CameraSubscriber()
        self.video_ros2_thread = Ros2Thread(self.camera_subscriber)
        self.video_ros2_thread.update_signal.connect(self.update_video)
        self.video_ros2_thread.start()
        
        # 尺寸
        self.scene_width = 1280-4
        self.scene_height = 630-4
        self.template_width = None
        self.template_height = None

        # 初始化绘图区域
        self.scene = QGraphicsScene()
        self.graphicsView.setScene(self.scene)
        self.scene.setSceneRect(0, 0, self.scene_width, self.scene_height)

        # 初始化模板图片
        self.background_item = None
        
        # 初始化
        self.current_mode = None
        self.temp_points = []
        self.temp_item = None
        self.calibration_info_flag = 1

        self.type = ''
        self.displacement = (None, None)
        self.opt_time = 1.000
        self.obj_height = 1.000
        self.EN = 1
        self.row_count = -1
        self.last_x = None
        self.last_y = None
        self.point_line_item = []
        self.click_temp_item = []
        self.points_pos = []

        self.start_point = None
        self.end_point = None
        self.cornor_selected = None
        self.angle = 0
        self.is_drawing = False
        self.drawonepoint = False
        self.is_square_finalized = False
        self.selected = False
        self.dragging = False
        self.current_polygon_item = None
        self.x = [None, None, None, None]
        self.y = [None, None, None, None]
        self.cornor_angle = [None, None, None, None]
        self.R = None
        self.x0 = None
        self.y0 = None
        self.cx = None
        self.cy = None
        self.square_part_item = []
        self.move_step = 5
        self.rotate_step = 2

        self.user_editing = False  # 标志位
        self.selected_row = None
        self.selected_col = None

        self.product_connection_path = "./gluemaster_files/connections.csv"
        self.product_EN = None
        self.product_name = ''
        self.product_rough_image_path = ''
        self.product_accurate_image_path = ''
        self.product_method_path = ''
        self.selected_product = None
        self.user_editing_product = False
        self.selected_row_product = None
        self.selected_col_product = None
        self.product_rough_image_path_temp = ''
        self.product_accurate_image_path_temp = ''

        self.enabled_products = []
        self.mv_status_flag = 0
        self.scale = None
        self.calibration_info = [
            "         请将准心对准参考点，按下回车时此位置将被保存为粗识别时的位置！",
            "         请将准心对准参考点，按下回车时将确定精识别摄像头的相对位置！",
            "         请将准心对准参考点，按下回车时将确定胶枪头的相对位置！",
        ]
        self.camera_parameter = []
        self.which_camera_is_editing = None
        self.speed_flag = 1

        self.product_num_dict = dict()

        # 槽信号
        self.btn_Start.clicked.connect(self.start_callback)
        self.btn_Stop.clicked.connect(self.stop_callback)
        self.btn_Reset.clicked.connect(self.reset_callback)

        self.btnAdd.clicked.connect(self.dialog1_func)
        self.btnDelete.clicked.connect(self.delete_row)
        self.btnAdd.setEnabled(False)
        self.btnDelete.setEnabled(False)

        self.btnAddProduct.clicked.connect(self.data_interface.add_product)
        self.btnExport.clicked.connect(self.data_interface.export_product)
        self.btnEdit.clicked.connect(self.data_interface.edit_product)
        self.btnRestore.clicked.connect(self.data_interface.restore_product)
        self.btnSave.clicked.connect(self.data_interface.save_product)
        self.btnDeleteProduct.clicked.connect(self.data_interface.delete_product)
        self.btnExport.setEnabled(False)
        self.btnRestore.setEnabled(False)
        self.btnSave.setEnabled(False)
        
        self.btn_Calibration.clicked.connect(self.calibration_callback)
        self.btn_test_rough.clicked.connect(self.rough_test_callback)
        self.btn_test_accurate.clicked.connect(self.accurate_test_callback)
        self.btn_Ruler_accurate.clicked.connect(self.accurate_ruler_callback)
        self.btn_Ruler_rough.clicked.connect(self.rough_ruler_callback)

        self.pushButton_roughpos.clicked.connect(self.move2roughpos_callback)
        self.pushButton_accuratepos.clicked.connect(self.move2accuratepos_callback)
        self.tabWidget.currentChanged.connect(self.page_change_callback)

        self.tableWidget.setSelectionMode(QAbstractItemView.SingleSelection)# 只允许单选
        self.tableWidget_product.setSelectionMode(QAbstractItemView.SingleSelection)

        self.tableWidget.cellClicked.connect(self.table_cell_clicked)
        self.tableWidget_product.cellClicked.connect(self.product_table_cell_clicked)

        self.tableWidget.cellDoubleClicked.connect(self.on_user_double_clicked)# 用户双击开始编辑
        self.tableWidget.cellChanged.connect(self.on_cell_changed)# 用户编辑结束

        self.tableWidget_product.cellDoubleClicked.connect(self.on_user_double_clicked_product)# 用户双击开始编辑
        self.tableWidget_product.cellChanged.connect(self.on_cell_changed_product)# 用户编辑结束

        self.tableWidget.itemChanged.connect(lambda: self.tableWidget.resizeColumnsToContents())
        self.tableWidget.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸
        self.tableWidget_product.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
        self.tableWidget_product.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸

        # 启用鼠标跟踪
        self.graphicsView.setMouseTracking(True)
        self.graphicsView.viewport().installEventFilter(self)

        self.textEdit.setReadOnly(True)
        self.textEdit_2.setReadOnly(True)
        self.textEdit_3.setReadOnly(True)
        self.textEdit_5.setReadOnly(True)
        self.textEdit_info_p3.setReadOnly(True)

        self.tabWidget.setCurrentIndex(0)
        
        self.tableWidget_number.setEditTriggers(QTableWidget.NoEditTriggers)

        validator = QDoubleValidator()
        validator.setBottom(0)
        self.lineEdit_length.setValidator(validator)
        # self.lineEdit_length.setText("")

        self.horizontalSlider_fps.setRange(0, 60)
        self.horizontalSlider_exposure.setRange(50, 10000)
        self.horizontalSlider_focus.setRange(0, 1022)

        self.horizontalSlider_fps.sliderReleased.connect(self.update_fps)
        self.horizontalSlider_exposure.sliderReleased.connect(self.update_exposure)
        self.horizontalSlider_focus.sliderReleased.connect(self.update_focus)
        self.checkBox_autoexposure.clicked.connect(self.update_auto_exposure)
        self.checkBox_autofocus.clicked.connect(self.update_auto_focus)

        self.radioButton_rough.toggled.connect(self.parameter_change_camera)
        self.radioButton_rough.setChecked(True)
        
        self.read_gluemaster_files()

    def update_label(self):
        if self.ui_subscriber_node.speed is not None:
            speed = self.ui_subscriber_node.speed
            if speed < 0 and self.speed_flag:
                self.speed_flag = 0
                self.write_to_textEdit("        传送带速度过快，系统停止运行！")
                self.label_beltspeed.setText("传送带的速度为 : 错误！")
            if speed>=0 and self.tabWidget.currentIndex() == 0:
                self.label_beltspeed.setText(f"传送带的速度为 : {speed:.3f} mm/s")

    def update_video(self):
        if self.camera_subscriber.current_frame is not None and self.tabWidget.currentIndex() == 0:
            # print(time.time())
            frame = self.camera_subscriber.current_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_video.size(), Qt.KeepAspectRatio)
            self.label_video.setPixmap(pixmap)
            self.label_video.setText("")
        elif self.camera_subscriber.current_frame is not None and self.tabWidget.currentIndex() == 2:
            frame = self.camera_subscriber.current_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_video_2.size(), Qt.KeepAspectRatio)
            if self.ui_subscriber_node.calibration_flag!=3:
                pixmap = self.draw_crosshair(pixmap)
                self.write_to_textEdit_3(self.calibration_info[self.ui_subscriber_node.calibration_flag])
            if self.calibration_info_flag:
                self.calibration_info_flag = 0
                self.write_to_textEdit_3("        ")
            self.label_video_2.setPixmap(pixmap)
            self.label_video_2.setText("")
        elif self.camera_subscriber.current_frame is not None and self.tabWidget.currentIndex() == 3:
            frame = self.camera_subscriber.current_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(self.label_video_3.size(), Qt.KeepAspectRatio)
            self.label_video_3.setPixmap(pixmap)
            self.label_video_3.setText("")
        else:
            self.label_video.setText("Not Connected!")
            self.label_video_2.setText("Not Connected!")
            self.label_video_3.setText("Not Connected!")

    def draw_crosshair(self, pixmap):
        """在 QPixmap 中心绘制十字准心"""
        painter = QPainter(pixmap)
        range, line_w = 50, 2
        pen = QPen(QColor(255, 0, 0, 180), line_w)
        painter.setPen(pen)
        width = pixmap.width()
        height = pixmap.height()
        center_x = width // 2
        center_y = height // 2
        painter.drawLine(center_x-range, center_y, center_x+range, center_y)
        painter.drawLine(center_x, center_y-range, center_x, center_y+range)
        painter.end()
        return pixmap

    def read_gluemaster_files(self):
        self.tableWidget_product.clearContents()
        self.tableWidget_product.setRowCount(0)
        file_path = "./gluemaster_files/connections.csv"
        with open(file_path, 'r', newline='', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            rowdata = list(reader)
            # print("-", rowdata)
            rowdata.pop(0)
            # print("---", rowdata)
            row = len(rowdata)
            # print("-----", row)
            self.data_interface.box = []
            temp = dict()
            for i in range(row):
                row_position = self.tableWidget_product.rowCount()  # 获取当前行数
                self.tableWidget_product.insertRow(row_position)  # 插入新行
                self.write_to_product_cells(i,0,QTableWidgetItem(rowdata[i][1]))
                self.write_to_product_cells(i,1,eval(rowdata[i][0]))
                # self.data_interface.box[i].setEnabled(False)
                temp[rowdata[i][1]] = 0
            self.product_num_dict = {**temp, **self.product_num_dict}
            print(self.product_num_dict)
        self.read_enabled_products()

    def read_enabled_products(self):
        self.tableWidget_number.clearContents()
        self.tableWidget_number.setRowCount(0)
        file_path = "./gluemaster_files/connections.csv"
        with open(file_path, 'r', encoding='utf-8') as csv_file:
            reader = csv.reader(csv_file)
            rowdata = list(reader)
            rowdata.pop(0)
            self.enabled_products = []
            # k = 0
            for i in range(len(rowdata)):
                # print(i, rowdata[i][0])
                if rowdata[i][0] == "1":
                    # print("ok")
                    self.enabled_products.append(rowdata[i][1])
                    # self.enabled_products[k].append(0)
                    # k+=1
            # print(self.enabled_products)
            for i in range(len(self.enabled_products)):
                row_position = self.tableWidget_number.rowCount()  # 获取当前行数
                self.tableWidget_number.insertRow(row_position)  # 插入新行
                self.write_to_enabled_product_cells(i,0,QTableWidgetItem(self.enabled_products[i]))
                self.write_to_enabled_product_cells(i,1,QTableWidgetItem(str(self.product_num_dict[self.enabled_products[i]])))
                print("enabled_products", self.enabled_products)
    
    def update_product_num(self, pdt_name):
        self.product_num_dict[pdt_name] += 1
        try:
            index = self.enabled_products.index(pdt_name)
            self.write_to_enabled_product_cells(index,1,QTableWidgetItem(str(self.product_num_dict[self.enabled_products[index]])))
            print(f"{pdt_name} 在列表中的索引位置是 {index}")
        except ValueError:
            print(f"{pdt_name} 不在列表中")

    def start_callback(self):
        self.led('red', 0)
        self.write_to_textEdit("         发送开始运行指令！")
        self.write_to_textEdit("         发送开始运行指令！")
        self.guiclient_node.send_request('start')
        self.led('green', 0)
        self.write_to_textEdit("         系统开始运行！")

    def stop_callback(self):
        self.led('red', 0)
        self.write_to_textEdit("         发送停止运行指令！")
        self.write_to_textEdit("         发送停止运行指令！")
        time.sleep(1)
        self.guiclient_node.send_request('stop')
        self.led('green', 0)
        self.write_to_textEdit("         系统停止运行！")
        self.speed_flag = 1

    def reset_callback(self):
        self.led('red', 0)
        self.write_to_textEdit("         发送复归运行指令！")
        self.write_to_textEdit("         发送复归运行指令！")
        self.guiclient_node.send_request('reset')
        self.led('green', 0)
        self.write_to_textEdit("         系统复位完成！")

    def calibration_callback(self):
        self.led('red')
        self.write_to_textEdit_3("         开始系统校准！")
        self.write_to_textEdit_3("         开始系统校准！")
        self.guiclient_node.send_request('calibration')
        self.data_interface.clear_all_tpl()
        self.write_to_textEdit_3("         系统校准完成！")
        self.led('green')
        self.calibration_info_flag = 1

    def rough_test_callback(self):
        self.led('red')
        self.write_to_textEdit_3("         尝试匹配粗模板!")
        self.write_to_textEdit_3("         尝试匹配粗模板!")
        self.guiclient_node.send_request('changecam',0)
        self.guiclient_node.send_request('move2roughpos')
        self.guiclient_node.send_request('test', 0)
        self.write_to_textEdit_3("         模板测试完成！")
        self.led('green')

    def accurate_test_callback(self):
        self.led('red')
        self.write_to_textEdit_3("         尝试匹配精模板！")
        self.write_to_textEdit_3("         尝试匹配精模板！")
        self.guiclient_node.send_request('changecam',1)
        time.sleep(1)
        self.guiclient_node.send_request('move2accuratepos')
        self.guiclient_node.send_request('test', 1)
        self.guiclient_node.send_request('changecam',0)
        self.write_to_textEdit_3("         模板测试完成！")
        self.led('green')

    def accurate_ruler_callback(self):
        self.led('red')
        self.write_to_textEdit_3("         开始测定精识别摄像头标尺！")
        self.write_to_textEdit_3("         开始测定精识别摄像头标尺！")
        sl = self.get_chess_board_side_length()
        if sl is not None:
            self.guiclient_node.send_request('changecam',1)
            self.guiclient_node.send_request('move2accuratepos')
            time.sleep(1)
            self.guiclient_node.send_request('ruler', 1)
            self.guiclient_node.send_request('changecam',0)
            self.guiclient_node.send_request('move2roughpos')
            self.data_interface.clear_all_tpl()
            self.write_to_textEdit_3("         精识别摄像头标尺测定成功！")
        self.led('green')
        self.data_interface.get_ruler()

    def rough_ruler_callback(self):
        self.led('red')
        self.write_to_textEdit_3("         开始测定粗识别摄像头标尺！")
        self.write_to_textEdit_3("         开始测定粗识别摄像头标尺！")
        sl = self.get_chess_board_side_length()
        if sl is not None:
            self.guiclient_node.send_request('move2roughpos')
            self.guiclient_node.send_request('ruler', 0)
            self.data_interface.clear_all_tpl()
            self.write_to_textEdit_3("         粗识别摄像头标尺测定成功！")
        self.led('green')

    def move2roughpos_callback(self):
        self.led("red", 3)
        self.guiclient_node.send_request('move2roughpos')
        self.which_camera_is_editing = 0
        self.guiclient_node.send_request('changecam', self.which_camera_is_editing)
        self.radioButton_accurate.setChecked(bool(self.which_camera_is_editing))
        self.led('green', 3)

    def move2accuratepos_callback(self):
        self.led("red", 3)
        self.guiclient_node.send_request('move2accuratepos')
        self.which_camera_is_editing = 1
        self.guiclient_node.send_request('changecam', self.which_camera_is_editing)
        self.radioButton_accurate.setChecked(bool(self.which_camera_is_editing))
        self.led('green', 3)

    def page_change_callback(self):
        index = self.tabWidget.currentIndex()
        print(f"切换到标签页 {index}_1")
        if index == 3:
            self.radioButton_accurate.setChecked(bool(self.which_camera_is_editing))
            print(f"切换到标签页 {index}_2")

    def get_chess_board_side_length(self):
        input = self.lineEdit_length.text()
        if not input:
            self.write_to_textEdit_3("         棋盘格方格边长输入不能为空！请重新输入正确的值！")
            return None
        side_length = round(eval(input)*100)
        if side_length <= 0:
            self.write_to_textEdit_3("         棋盘格方格边长输入应该为正！请重新输入正确的值！")
            return None
        print(side_length)
        return side_length
    
    def led(self, color='green', page=2):
        if page == 2:
            self.label_led.setStyleSheet(f"background-color: {color};border-radius: 30px;")
            self.label_led.update()
            self.parameter_change_camera
        elif page == 0:
            self.label_led_2.setStyleSheet(f"background-color: {color};border-radius: 30px;")
            self.label_led_2.update()
        elif page == 3:
            self.label_led_3.setStyleSheet(f"background-color: {color};border-radius: 30px;")
            self.label_led_3.update()
        QtWidgets.QApplication.processEvents()

    def write_to_textEdit(self, text):
        self.textEdit.setPlainText("提示信息：\n"+str(text))
        self.textEdit.update()
        QtWidgets.QApplication.processEvents()

    def write_to_textEdit_2(self, text):
        self.textEdit_2.setPlainText("提示信息：\n"+str(text))
        self.textEdit.update()
        QtWidgets.QApplication.processEvents()

    def write_to_textEdit_3(self, text):
        self.textEdit_info_p3.setPlainText("提示信息：\n"+str(text))
        self.textEdit.update()
        QtWidgets.QApplication.processEvents()

    def set_init_video_parameter(self):
        self.camera_parameter = []
        self.camera_parameter = gc.GetCSVlist(gc.gluemasterfilepath+'/camparam.csv')
        self.horizontalSlider_fps.setValue(self.camera_parameter[self.which_camera_is_editing][0])
        self.horizontalSlider_exposure.setValue(self.camera_parameter[self.which_camera_is_editing][6])
        self.horizontalSlider_focus.setValue(self.camera_parameter[self.which_camera_is_editing][4])
        self.label_fps.setText(f"{self.camera_parameter[self.which_camera_is_editing][0]}")
        self.label_exposure.setText(f"{self.camera_parameter[self.which_camera_is_editing][6]}")
        self.label_focus.setText(f"{self.camera_parameter[self.which_camera_is_editing][4]}")

        if self.camera_parameter[self.which_camera_is_editing][5] == 1:
            self.checkBox_autoexposure.setChecked(False)
            self.horizontalSlider_exposure.setEnabled(True)
        elif self.camera_parameter[self.which_camera_is_editing][5] == 3:
            self.checkBox_autoexposure.setChecked(True)
            self.horizontalSlider_exposure.setEnabled(False)
        if self.camera_parameter[self.which_camera_is_editing][3] == 0:
            self.checkBox_autofocus.setChecked(False)
            self.horizontalSlider_focus.setEnabled(True)
        elif self.camera_parameter[self.which_camera_is_editing][3] == 1:
            self.checkBox_autofocus.setChecked(True)
            self.horizontalSlider_focus.setEnabled(False)

    def update_fps(self):
        self.camera_parameter[self.which_camera_is_editing][0] = self.horizontalSlider_fps.value()
        self.label_fps.setText(f"{self.camera_parameter[self.which_camera_is_editing][0]}")
        self.save_camera_parameter()

    def update_exposure(self):
        self.camera_parameter[self.which_camera_is_editing][6] = self.horizontalSlider_exposure.value()
        self.label_exposure.setText(f"{self.camera_parameter[self.which_camera_is_editing][6]}")
        self.save_camera_parameter()

    def update_focus(self):
        self.camera_parameter[self.which_camera_is_editing][4] = self.horizontalSlider_focus.value()
        self.label_focus.setText(f"{self.camera_parameter[self.which_camera_is_editing][4]}")
        self.save_camera_parameter()

    def update_auto_exposure(self):
        temp = self.checkBox_autoexposure.isChecked()
        if temp:
            self.camera_parameter[self.which_camera_is_editing][5] = 3
            self.horizontalSlider_exposure.setEnabled(False)
        else:
            self.camera_parameter[self.which_camera_is_editing][5] = 1
            self.horizontalSlider_exposure.setEnabled(True)
        self.save_camera_parameter()

    def update_auto_focus(self):
        temp = self.checkBox_autofocus.isChecked()
        if temp:
            self.camera_parameter[self.which_camera_is_editing][3] = 1
            self.horizontalSlider_focus.setEnabled(False)
        else:
            self.camera_parameter[self.which_camera_is_editing][3] = 0
            self.horizontalSlider_focus.setEnabled(True)
        self.save_camera_parameter()

    def save_camera_parameter(self):
        self.led("red", 3)
        gc.WriteCSVlist(self.camera_parameter)
        print(self.camera_parameter)
        self.guiclient_node.send_request('changecam', self.which_camera_is_editing)
        self.led("green", 3)

    def parameter_change_camera(self):
        if self.radioButton_rough.isChecked():
            self.guiclient_node.send_request('changecam', 0)
            self.which_camera_is_editing = 0
            print('rough_camera',self.which_camera_is_editing)
            self.set_init_video_parameter()
        elif self.radioButton_accurate.isChecked():
            self.guiclient_node.send_request('changecam', 1)
            self.which_camera_is_editing = 1
            print('accurate_camera',self.which_camera_is_editing)
            self.set_init_video_parameter()

    def write_to_item_cells(self, write_row, write_col, write_item, flag=1):
        if flag:
            self.tableWidget.setItem(write_row,write_col,write_item)
        rows = self.tableWidget.rowCount()
        for row in range(rows):
            for col in range(7):
                item = self.tableWidget.item(row, col)
                if item:
                    item.setTextAlignment(Qt.AlignCenter)
        self.tableWidget.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸

    def write_to_product_cells(self, write_row, write_col, write_item, flag=1):
        if flag:
            if write_col==0:
                self.tableWidget_product.setItem(write_row,0,write_item)
            elif write_col==1:
                combo = QComboBox()
                self.data_interface.box.append(combo)
                combo.addItem("启用", True)
                combo.addItem("不启用", False)
                combo.setStyleSheet("""
                    QComboBox {
                        text-align: center;           /* 下拉框文字居中 */
                        padding-left: 60px;          /* 调整左边距（可选） */
                    }
                    # QComboBox QAbstractItemView {
                    #     padding-left: 60px;          /* 调整左边距（可选） */
                    #     padding-right: 60px;          /* 调整左边距（可选） */
                    # }
                    """)
                combo.setCurrentIndex(not write_item) #[启用, 不启用]
                self.tableWidget_product.setCellWidget(write_row, 1, combo)
                combo.view().setEnabled(False)

                selected_value = combo.currentData()
                # print(selected_value)  # 输出 True 或 False
        rows = self.tableWidget_product.rowCount()
        for row in range(rows):
            for col in range(2):
                item = self.tableWidget_product.item(row, col)
                if item:
                    item.setTextAlignment(Qt.AlignCenter)
                    # print(row,col)
        self.tableWidget_product.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
        self.tableWidget_product.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸

    def write_to_enabled_product_cells(self, write_row, write_col, write_item, flag=1):
        if flag:
            self.tableWidget_number.setItem(write_row,write_col,write_item)
        rows = self.tableWidget_number.rowCount()
        for row in range(rows):
            for col in range(2):
                item = self.tableWidget_number.item(row, col)
                if item:
                    item.setTextAlignment(Qt.AlignCenter)
        self.tableWidget_number.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
        self.tableWidget_number.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸

    def dialog1_func(self): # 小窗1,选择画图类型和颜色
        if self.template_height and self.template_width and self.mv_status_flag==0:
            self.add_row()
            dialog1 = Dialog_painter_set(window=self)
            if dialog1.exec_() == QDialog.Accepted:
                # self.mv_status_flag += 1
                self.row_count = self.tableWidget.rowCount()-1
                self.modenum = dialog1.get_selected_option()
                if self.modenum == 1:
                    self.type = 'point'
                    self.set_mode(self.type)
                    self.write_to_item_cells(self.row_count,0,QTableWidgetItem(self.type))
                elif self.modenum == 2:
                    self.type = 'line'
                    self.set_mode(self.type)
                    self.write_to_item_cells(self.row_count,0,QTableWidgetItem(self.type))
                elif self.modenum == 3:
                    self.type = 'circle'
                    self.set_mode(self.type)
                    self.write_to_item_cells(self.row_count,0,QTableWidgetItem(self.type))
                self.btnDelete.setEnabled(False)

    def diaglog2_func(self, row, times=1): # 小窗2,设置点胶高度和时间
        # if self.mv_status_flag == 1:
        # dialog2 = Dialog_mv_set(window=self)
        # if dialog2.exec_() == QDialog.Accepted:
        #     # self.mv_status_flag += 1
        #     self.opt_time, self.obj_height = dialog2.get_num_value()
        #     self.opt_time, self.obj_height = round(self.opt_time,3), round(self.obj_height,3)
        #     self.write_to_item_cells(row,4,QTableWidgetItem(str(self.opt_time)))
        #     self.write_to_item_cells(row,5,QTableWidgetItem(str(self.obj_height)))
        #     # self.write_to_item_cells(row,6,QTableWidgetItem(str(self.EN)))
        #     if times != 2:
        #         self.update_movement()
        #         self.delete_all_item()
        #         self.draw_all_item()
        if self.type == 'circle':
            self.opt_time = 2
        else:
            self.opt_time = 0.5
        if self.type == 'movement':
            self.obj_height = 20
        else:
            self.obj_height = 5
        self.write_to_item_cells(row,4,QTableWidgetItem(str(self.opt_time)))
        self.write_to_item_cells(row,5,QTableWidgetItem(str(self.obj_height)))
        self.btnDelete.setEnabled(True)
        if times != 2:
            self.update_movement()
            self.delete_all_item()
            self.draw_all_item()

    def round_tuple(self, t):
        return tuple(self.round_tuple(i) if isinstance(i, tuple) else round(i, 3) for i in t)
    
    def product_table_cell_clicked(self):
        self.user_editing_product = False
        selected_items = self.tableWidget_product.selectedItems()
        if selected_items:
            item = selected_items[0]
            # item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            if self.data_interface.is_editting_product == False:
                self.selected_product = item.row()

    def on_user_double_clicked_product(self):
        self.user_editing_product = True
        # self.data_interface.is_editting_product = True
        # self.btnRestore.setEnabled(True)
        # self.btnSave.setEnabled(True)
        selected_items = self.tableWidget_product.selectedItems()
        if selected_items:
            item = selected_items[0]
            self.selected_row_product, self.selected_col_product = item.row(), item.column()
            # if self.selected_row_product 
            if self.selected_row_product == self.selected_product and self.selected_col_product == 0 and self.data_interface.is_editting_product:
                self.data_interface.old_name = self.product_name
                self.data_interface.set_product_name(type="edit")
                if self.data_interface.name_state == 'accept':
                    self.write_to_product_cells(self.selected_row_product,self.selected_col_product,
                                                QTableWidgetItem(self.product_name))
                    self.data_interface.name_edited = True
                    self.data_interface.new_name = self.product_name
        self.tableWidget_product.setEditTriggers(QTableWidget.NoEditTriggers)

    def on_cell_changed_product(self):
        self.user_editing_product = False

    def table_cell_clicked(self): # 选中表格
        self.user_editing = False
        selected_items = self.tableWidget.selectedItems()
        self.del_all_bold()
        point_w, line_w = 10, 4
        if selected_items:
            item = selected_items[0]
            row, col = item.row(), item.column()
            # if col in [0, 3] or (self.tableWidget.item(row, 0).text()=='movement' and col in [1, 2]):
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            type = self.tableWidget.item(row,0).text()
            pos = self.points_pos[row]
            if type == 'point':
                point = self.scene.addEllipse(pos[0][0]-point_w/2, pos[0][1]-point_w/2, point_w, point_w,QPen(point_color),QBrush(point_color))
                self.click_temp_item.extend([point])
                point = None
            elif type == 'circle':
                x, y, cx, cy = pos[0][0], pos[0][1], pos[2][0], pos[2][1]
                radius = math.sqrt((cx-x)**2+(cy-y)**2)
                point = self.scene.addEllipse(x-point_w/2, y-point_w/2, point_w, point_w, QPen(point_color),QBrush(point_color))
                circle = QGraphicsEllipseItem(cx-radius, cy-radius, 2*radius, 2*radius)
                circle.setPen(QPen(line_color, line_w))
                self.scene.addItem(circle)
                self.click_temp_item.extend([point, circle])
                point, circle = None, None
            elif type == 'line':
                x1, y1, x2, y2 = pos[0][0], pos[0][1], pos[1][0], pos[1][1]
                line = QGraphicsLineItem(x1, y1, x2, y2)
                point1 = self.scene.addEllipse(x1-point_w/2, y1-point_w/2, point_w, point_w, QPen(point_color),QBrush(point_color))
                point2 = self.scene.addEllipse(x2-point_w/2, y2-point_w/2, point_w, point_w, QPen(point_color),QBrush(point_color))
                line.setPen(QPen(line_color, line_w))
                self.scene.addItem(line)
                self.click_temp_item.extend([point1, point2, line])
                point1, point2, line = None, None, None
            elif type == 'movement':
                dash_line = QGraphicsLineItem(pos[0][0], pos[0][1], pos[1][0], pos[1][1])
                dash_line.setPen(QPen(line_color, line_w, Qt.DashLine))
                self.scene.addItem(dash_line)
                self.click_temp_item.extend([dash_line])
                dash_line = None
        self.write_to_item_cells(0,0,0,0)

    def on_user_double_clicked(self):
        if self.data_interface.is_editting_product:
            self.user_editing = True
        else:
            self.tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)
        # self.data_interface.is_editting_product = True
        # self.btnRestore.setEnabled(True)
        # self.btnSave.setEnabled(True)
        selected_items = self.tableWidget.selectedItems()
        if selected_items:
            item = selected_items[0]
            self.selected_row, self.selected_col = item.row(), item.column()
            if self.data_interface.is_editting_product or self.data_interface.is_adding_product:
                if self.tableWidget.item(self.selected_row, 0).text()!='movement' and self.selected_col in [1, 2]:
                    dialog = Dialog_point_reset(window=self)
                    dialog.exec_()
                    if dialog.state == 'accept':
                        dialog.state = ''
                        x, y = dialog.get_text()
                        new_point = (self.data_interface.real2pixel(eval(x), 'x'), self.data_interface.real2pixel(eval(y), 'y'))
                        print("new_point", new_point)
                        tp = self.round_tuple(new_point)
                        print("tp", tp)
                        self.write_to_item_cells(self.selected_row, self.selected_col, QTableWidgetItem(str(tp)))
                elif self.selected_col in [4, 5]:
                    if self.selected_col in [4]:
                        dialog = Dialog_parameter_reset(window=self, type='time')
                    elif self.selected_col in [5]:
                        dialog = Dialog_parameter_reset(window=self, type='height')
                    dialog.exec_()
                    if dialog.state == 'accept':
                        dialog.state = ''
                        pmt = dialog.get_text()
                        new_parameter = eval(pmt)
                        self.write_to_item_cells(self.selected_row, self.selected_col, QTableWidgetItem(str(new_parameter)))
        self.tableWidget.setEditTriggers(QTableWidget.NoEditTriggers)

    def on_cell_changed(self):
        if self.user_editing: 
            self.user_editing = False
            if self.selected_col==1 or self.selected_col==2:
                self.del_all_bold()
                # print("yes")
                row_count = self.tableWidget.rowCount()
                print("row1876-test: ", row_count, self.selected_row, '------')
                type = self.tableWidget.item(self.selected_row,0).text()
                new_point = eval(self.tableWidget.item(self.selected_row,self.selected_col).text())
                print(new_point)
                if type == 'point':
                    self.points_pos[self.selected_row][0] = new_point
                    self.points_pos[self.selected_row][1] = new_point
                    self.points_pos[self.selected_row-1][1] = new_point
                    tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                    self.write_to_item_cells(self.selected_row,1,QTableWidgetItem(str(tp)))
                    self.write_to_item_cells(self.selected_row,2,QTableWidgetItem(str(tp)))
                    self.write_to_item_cells(self.selected_row-1,2,QTableWidgetItem(str(tp)))
                    if self.selected_row != row_count-1:
                        self.points_pos[self.selected_row+1][0] = new_point
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                        self.write_to_item_cells(self.selected_row+1,1,QTableWidgetItem(str(tp)))
                elif type == 'circle':
                    if self.selected_col == 1:
                        self.points_pos[self.selected_row][0] = new_point
                        self.points_pos[self.selected_row][1] = new_point
                        self.points_pos[self.selected_row-1][1] = new_point
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                        self.write_to_item_cells(self.selected_row,1,QTableWidgetItem(str(tp)))
                        self.write_to_item_cells(self.selected_row-1,2,QTableWidgetItem(str(tp)))
                        if self.selected_row != row_count-1:
                            self.points_pos[self.selected_row+1][0] = new_point
                            tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                            self.write_to_item_cells(self.selected_row+1,1,QTableWidgetItem(str(tp)))
                    elif self.selected_col == 2:
                        self.points_pos[self.selected_row][2] = new_point
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                        self.write_to_item_cells(self.selected_row,2,QTableWidgetItem(str(tp)))
                elif type == 'line':
                    if self.selected_col == 1:
                        self.points_pos[self.selected_row][0] = new_point
                        self.points_pos[self.selected_row-1][1] = new_point
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                        self.write_to_item_cells(self.selected_row,1,QTableWidgetItem(str(tp)))
                        self.write_to_item_cells(self.selected_row-1,2,QTableWidgetItem(str(tp)))
                    elif self.selected_col == 2:
                        self.points_pos[self.selected_row][1] = new_point
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                        self.write_to_item_cells(self.selected_row,2,QTableWidgetItem(str(tp)))
                        if self.selected_row != row_count-1:
                            self.points_pos[self.selected_row+1][0] = new_point
                            tp = self.round_tuple(self.data_interface.tuple_pixel2real(new_point))
                            self.write_to_item_cells(self.selected_row+1,1,QTableWidgetItem(str(tp)))
            elif self.selected_col==4 or self.selected_col==5:
                self.tableWidget.horizontalHeader().setStretchLastSection(True)  # 最后一列填满剩余空间
                self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  # 所有列自动平均拉伸
            for i in self.points_pos:
                print(i)
            self.update_displacement()
            self.update_movement()
            self.delete_all_item()
            self.draw_all_item()

    def update_displacement(self):
        row_count = self.tableWidget.rowCount()
        for i in range (row_count):
            type = self.tableWidget.item(i,0).text()
            if type == 'point':
                continue
            elif type == 'circle':
                self.displacement = (self.points_pos[i][2][0]-self.points_pos[i][0][0],
                                     self.points_pos[i][2][1]-self.points_pos[i][0][1])
                # self.displacement = self.round_tuple(self.displacement)
                tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                self.write_to_item_cells(i,3,QTableWidgetItem(str(tp)))
            else:
                self.displacement = (self.points_pos[i][1][0]-self.points_pos[i][0][0],
                                     self.points_pos[i][1][1]-self.points_pos[i][0][1])
                self.displacement = self.round_tuple(self.displacement)
                tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                self.write_to_item_cells(i,3,QTableWidgetItem(str(tp)))
                
    def update_movement(self):
        init_central_point = (self.template_width/2, self.template_height/2)
        temp = []
        if self.points_pos:
            first_start = self.points_pos[0][0]
            if init_central_point != first_start:
                temp.append([init_central_point, first_start])
                self.displacement = (first_start[0]-init_central_point[0], first_start[1]-init_central_point[1])
                self.displacement = self.round_tuple(self.displacement)
                self.tableWidget.insertRow(0)
                self.type = 'movement'
                self.write_to_item_cells(0,0,QTableWidgetItem(self.type))
                tp = self.round_tuple(self.data_interface.tuple_pixel2real(init_central_point))
                self.write_to_item_cells(0,1,QTableWidgetItem(str(tp)))
                tp = self.round_tuple(self.data_interface.tuple_pixel2real(first_start))
                self.write_to_item_cells(0,2,QTableWidgetItem(str(tp)))
                tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                self.write_to_item_cells(0,3,QTableWidgetItem(str(tp)))
                self.EN = 0
                self.diaglog2_func(0, 2)
            temp.append(self.points_pos[0])
            for i in range(1, len(self.points_pos)):
                prev_end = temp[-1][1]
                curr_start = self.points_pos[i][0]
                curr_end = self.points_pos[i][1]
                if prev_end != curr_start:
                    temp.append([prev_end, curr_start])
                    self.displacement = (curr_start[0]-prev_end[0], curr_start[1]-prev_end[1])
                    self.displacement = self.round_tuple(self.displacement)
                    print("x = ",self.displacement)
                    self.tableWidget.insertRow(i)
                    self.type = 'movement'
                    self.write_to_item_cells(i,0,QTableWidgetItem(self.type))
                    tp = self.round_tuple(self.data_interface.tuple_pixel2real(prev_end))
                    self.write_to_item_cells(i,1,QTableWidgetItem(str(tp)))
                    tp = self.round_tuple(self.data_interface.tuple_pixel2real(curr_start))
                    self.write_to_item_cells(i,2,QTableWidgetItem(str(tp)))
                    tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                    self.write_to_item_cells(i,3,QTableWidgetItem(str(tp)))
                    self.EN = 0
                    self.diaglog2_func(i, 2)
                temp.append(self.points_pos[i])
        self.points_pos = temp
        # for i in temp:
        #     print(i)

    def special_process(self, en, pos):
        elem = en[pos]
        if elem:
            left = pos
            while left > 0 and en[left - 1] == 1:
                left -= 1
            right = pos
            while right < len(en) - 1 and en[right + 1] == 1:
                right += 1
            while left > 0 and en[left - 1] == 0:
                left -= 1
            while right < len(en) - 1 and en[right + 1] == 0:
                right += 1
            rows_to_delete = list(range(left, right+1))
            for i in sorted(rows_to_delete, reverse=True): # 逆序删除
                self.tableWidget.removeRow(i)  # 删除选中的行
                self.points_pos.pop(i)
        else:
            self.EN = 0
            self.diaglog2_func(pos, 2)
        self.update_movement()
        self.delete_all_item()
        self.draw_all_item()

    def delete_all_item(self):
        for i in range(len(self.point_line_item)):
            self.scene.removeItem(self.point_line_item[i])
        self.point_line_item = []

    def draw_all_item(self):
        for i in range(len(self.points_pos)):
            # print(i)
            pos = self.points_pos[i]
            type = self.tableWidget.item(i,0).text()
            if type == 'point':
                point = self.scene.addEllipse(pos[0][0]-2, pos[0][1]-2, 4, 4,QPen(point_color),QBrush(point_color))
                self.point_line_item.extend([point])
                point = None
            elif type == 'circle':
                x, y, cx, cy = pos[0][0], pos[0][1], pos[2][0], pos[2][1]
                radius = math.sqrt((cx-x)**2+(cy-y)**2)
                point = self.scene.addEllipse(x-2, y-2, 4, 4, QPen(point_color),QBrush(point_color))
                circle = QGraphicsEllipseItem(cx-radius, cy-radius, 2*radius, 2*radius)
                circle.setPen(QPen(line_color, 2))
                self.scene.addItem(circle)
                self.point_line_item.extend([point, circle])
                point, circle = None, None
            elif type == 'line':
                x1, y1, x2, y2 = pos[0][0], pos[0][1], pos[1][0], pos[1][1]
                line = QGraphicsLineItem(x1, y1, x2, y2)
                point1 = self.scene.addEllipse(x1-2, y1-2, 4, 4, QPen(point_color),QBrush(point_color))
                point2 = self.scene.addEllipse(x2-2, y2-2, 4, 4, QPen(point_color),QBrush(point_color))
                line.setPen(QPen(line_color, 2))
                self.scene.addItem(line)
                self.point_line_item.extend([point1, point2, line])
                point1, point2, line = None, None, None
            elif type == 'movement':
                dash_line = QGraphicsLineItem(pos[0][0], pos[0][1], pos[1][0], pos[1][1])
                dash_line.setPen(QPen(line_color, 1, Qt.DashLine))
                self.scene.addItem(dash_line)
                self.point_line_item.extend([dash_line])
                dash_line = None

    def add_row(self): # 添加行，点击add触发
        row_position = self.tableWidget.rowCount()  # 获取当前行数
        self.tableWidget.insertRow(row_position)  # 插入新行

    def delete_row(self): # 删除行，点击delete触发，同时删除绘制的路径
        selected_row = self.tableWidget.currentRow()  # 获取当前选中的行, 0,1,2...
        row_count = self.tableWidget.rowCount()
        if selected_row >= 0:
            self.del_all_bold()
            en = []
            for i in range(row_count):
                if self.tableWidget.item(i, 0).text() == 'movement':
                    en.append(0)
                else:
                    en.append(1)
                    # en.append(eval(self.tableWidget.item(i, 6).text()))
            self.special_process(en, selected_row)

    def del_all_bold(self): # 取消所有加粗
        for i in range(len(self.click_temp_item)):
            self.scene.removeItem(self.click_temp_item[i])
    
    def set_mode(self, mode):
        self.current_mode = mode
        self.temp_points = []
        self.remove_temp_item()
        # self.labelInfo.setText(f"模式：{mode} - 点击第一个点" if mode in ["line", "circle", "square"] else f"模式：{mode}")

    def clear_scene(self):
        # 只移除非背景项
        for item in self.scene.items():
            if item != self.background_item:
                self.scene.removeItem(item)
        # self.labelInfo.setText("已清除所有图形")

    # def startDrawing(self):
    #     self.is_drawing = True
    #     self.start_point = None
    #     self.end_point = None
    #     self.cornor_selected = None
    #     self.drawonepoint = False
    #     self.selected = False
    #     self.angle = 0
    #     self.is_square_finalized = False
    #     self.dragging = False
    #     self.set_mode('square')
    #     self.del_paint_square()
    
    # def stopDrawing(self):
    #     self.is_drawing = False
    #     self.start_point = None
    #     self.end_point = None
    #     self.cornor_selected = None
    #     self.drawonepoint = False
    #     self.selected = False
    #     self.angle = 0
    #     self.is_square_finalized = False
    #     self.dragging = False
    #     self.del_paint_square()

    def keyPressEvent(self, event):
        global flag
        if event.key() == Qt.Key_Escape:
            if self.current_mode == "line":
                # self.labelInfo.setText("直线模式结束，请重新选择模式或开始新的绘制")
                self.remove_temp_item()
                self.temp_points = []
                flag = 0
                self.EN = 1
                row_count = self.tableWidget.rowCount()
                self.tableWidget.removeRow(row_count - 1)
                self.update_movement()
                self.delete_all_item()
                self.draw_all_item()
                event.accept()

            if  self.current_mode == "square":
                self.scene.removeItem(self.temp)
                self.del_paint_square()
                self.stopDrawing()

            self.tableWidget.clearSelection()

        # if event.key() == Qt.Key_A and self.selected:
        #     self.square_move(-self.move_step, 0, 0)
        # elif event.key() == Qt.Key_D and self.selected:
        #     self.square_move(self.move_step, 0, 0)
        # elif event.key() == Qt.Key_W and self.selected:
        #     self.square_move(0, -self.move_step, 0)
        # elif event.key() == Qt.Key_S and self.selected:
        #     self.square_move(0, self.move_step, 0)

        # if event.key() == Qt.Key_V and self.selected:
        #     self.square_rotate(self.rotate_step)
        # if event.key() == Qt.Key_Z and self.selected:
        #     self.square_rotate(-self.rotate_step)

        else:
            super().keyPressEvent(event)

    def eventFilter(self, source, event):
        """处理鼠标事件"""
        global point_color, line_color, flag
        if source is self.graphicsView.viewport():
            # 鼠标移动事件处理
            if event.type() == QtCore.QEvent.MouseMove:
                pos = self.graphicsView.mapToScene(event.pos())
                x, y = int(pos.x()), int(pos.y())
                
                if self.template_width and self.template_height:
                    if not (0 <= x <= self.template_width and 0 <= y <= self.template_height):
                        return True
                
                # 直线模式预览
                if self.current_mode == "line" and len(self.temp_points) == 1:
                    self.remove_temp_item()
                    start_x, start_y = self.temp_points[0]
                    self.temp_item = QGraphicsLineItem(start_x, start_y, x, y)
                    self.temp_item.setPen(QPen(line_color, 2, Qt.DashLine))
                    self.scene.addItem(self.temp_item)
                
                # 圆模式三点预览
                elif self.current_mode == "circle" and len(self.temp_points) == 2:
                    self.remove_temp_item()
                    cx, cy, radius = self.calculate_circle(self.temp_points[0],self.temp_points[1],(x, y))
                    if radius is not None:
                        self.temp_item = QGraphicsEllipseItem(cx-radius, cy-radius, 2*radius, 2*radius)
                        self.temp_item.setPen(QPen(line_color, 2, Qt.DashLine))
                        self.scene.addItem(self.temp_item)

                # elif self.current_mode == 'square':
                # if self.drawonepoint:
                #     self.paint_square(self.x[0], self.y[0], x, self.y[0], x, y, self.x[0], y, 1, 0)
                # if self.is_square_finalized and self.dragging:
                #     self.square_move(x, y)
                # if self.cornor_selected:
                #     self.x[self.cornor_selected-1], self.y[self.cornor_selected-1] = x, y
                #     self.cornor_update(self.cornor_selected%2)
                #     self.square_stretch(self.cornor_selected-1)
                #     self.cornor_update(self.cornor_selected%2)
                #     self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 1)

            elif event.type() == QtCore.QEvent.MouseButtonRelease and self.is_square_finalized:
                time.sleep(0.01)
                if event.type() == QtCore.QEvent.MouseButtonRelease and self.is_square_finalized:
                    self.dragging = False
                    self.cornor_selected = None
                    self.cornor_update()

            # 鼠标点击事件处理
            elif event.type() == QtCore.QEvent.MouseButtonPress:
                if event.button() == Qt.LeftButton:
                    pos = self.graphicsView.mapToScene(event.pos())
                    x, y = int(pos.x()), int(pos.y())

                    if not (0 <= x <= self.template_width and 0 <= y <= self.template_height):
                        return True
                    
                    if self.x[0] and self.x[1] and self.is_square_finalized:
                        w = 8
                        if self.current_polygon_item.contains(QPointF(x, y)):
                            self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 2)
                            self.cornor_selected = None
                            if self.x[0]-w<=x<=self.x[0]+w and self.y[0]-w<=y<=self.y[0]+w:
                                self.cornor_selected = 1
                            elif self.x[1]-w<=x<=self.x[1]+w and self.y[1]-w<=y<=self.y[1]+w:
                                self.cornor_selected = 2
                            elif self.x[2]-w<=x<=self.x[2]+w and self.y[2]-w<=y<=self.y[2]+w:
                                self.cornor_selected = 3
                            elif self.x[3]-w<=x<=self.x[3]+w and self.y[3]-w<=y<=self.y[3]+w:
                                self.cornor_selected = 4
                            else:
                                self.dragging = True
                                self.selected = True
                                self.x0, self.y0 = x, y
                            print(self.cornor_selected)
                        else:
                            self.selected = False
                            self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 1)
            
                    # 点模式直接绘制
                    if self.current_mode == "point" and flag:
                        self.points_pos.append([(x, y), (x, y)])
                        point = self.scene.addEllipse(x-2, y-2, 4, 4, QPen(point_color),QBrush(point_color))
                        self.point_line_item.extend([point])
                        # self.labelInfo.setText(f"点坐标：({x}, {y})")
                        self.EN = 1
                        tp = self.round_tuple(self.data_interface.tuple_pixel2real((x, y)))
                        self.write_to_item_cells(self.row_count,1,QTableWidgetItem(str(tp)))
                        self.write_to_item_cells(self.row_count,2,QTableWidgetItem(str(tp)))
                        self.write_to_item_cells(self.row_count,3,QTableWidgetItem(str((0, 0))))
                        flag = 0
                        self.diaglog2_func(self.row_count)
                    
                    # 直线模式
                    elif self.current_mode == "line" and flag:
                        if len(self.temp_points) == 0:
                            self.temp_points.append((x, y))
                            point = self.scene.addEllipse(x-2, y-2, 4, 4,QPen(point_color),QBrush(point_color))
                            self.point_line_item.extend([point])
                            # self.labelInfo.setText(f"直线模式：已记录起点 ({x}, {y})，请点击终点")
                        else:
                            self.remove_temp_item()
                            x1, y1 = self.temp_points[-1]
                            self.points_pos.append([(x1, y1), (x, y)])
                            line = QGraphicsLineItem(x1, y1, x, y)
                            line.setPen(QPen(line_color, 2))
                            point = self.scene.addEllipse(x-2, y-2, 4, 4,QPen(point_color),QBrush(point_color))
                            self.scene.addItem(line)
                            self.point_line_item.extend([point, line])
                            # self.labelInfo.setText(f"直线：起点({x1}, {y1})，终点({x}, {y})")
                            self.temp_points = [(x, y)]
                            self.type = 'line'
                            self.displacement = (x-x1, y-y1)
                            self.displacement = self.round_tuple(self.displacement)
                            tp = self.round_tuple(self.data_interface.tuple_pixel2real((x1, y1)))
                            self.write_to_item_cells(self.row_count,1,QTableWidgetItem(str(tp)))
                            tp = self.round_tuple(self.data_interface.tuple_pixel2real((x, y)))
                            self.write_to_item_cells(self.row_count,2,QTableWidgetItem(str(tp)))
                            self.EN = 1
                            tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                            self.write_to_item_cells(self.row_count,3,QTableWidgetItem(str(tp)))
                            self.diaglog2_func(self.row_count, 2)
                            self.row_count = self.row_count + 1
                            self.add_row()
                            self.write_to_item_cells(self.row_count,0,QTableWidgetItem(str(self.type)))
                            self.point_line_item.extend([point])
                            self.EN = 1
                    # 圆模式
                    elif self.current_mode == "circle" and flag:
                        if len(self.temp_points) < 2:
                            self.temp_points.append((x, y))
                            if len(self.temp_points) == 1:
                                point = self.scene.addEllipse(x-2, y-2, 4, 4,QPen(point_color),QBrush(point_color))
                                self.point_line_item.extend([point])
                            # status_text = ["圆模式：请点击第二点","圆模式：请点击第三点"][len(self.temp_points)-1]
                            # self.labelInfo.setText(status_text)
                        else:
                            self.remove_temp_item()
                            cx, cy, radius = self.calculate_circle(self.temp_points[0],self.temp_points[1],(x, y))
                            cx, cy = round(cx,3), round(cy,3)
                            if radius is not None:
                                circle = QGraphicsEllipseItem(cx-radius, cy-radius, 2*radius, 2*radius)
                                self.points_pos.append([self.temp_points[0], self.temp_points[0], (cx, cy)])
                                circle.setPen(QPen(line_color, 2))
                                self.scene.addItem(circle)
                                self.point_line_item.extend([circle])
                                # self.labelInfo.setText(f"圆：圆心({cx}, {cy})，半径{radius}")
                                self.type = 'circle'
                                self.displacement = (cx-self.temp_points[0][0], cy-self.temp_points[0][1])
                                self.displacement = self.round_tuple(self.displacement)
                                tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.temp_points[0]))
                                self.write_to_item_cells(self.row_count,1,QTableWidgetItem(str(tp)))
                                tp = self.round_tuple(self.data_interface.tuple_pixel2real((cx, cy)))
                                self.write_to_item_cells(self.row_count,2,QTableWidgetItem(str(tp)))
                                tp = self.round_tuple(self.data_interface.tuple_pixel2real(self.displacement))
                                self.write_to_item_cells(self.row_count,3,QTableWidgetItem(str(tp)))
                                self.EN = 1
                                flag = 0
                                self.diaglog2_func(self.row_count)
                            else:
                                # self.labelInfo.setText("错误：三点共线，无法生成圆")
                                pass
                            self.temp_points = []

                    # 正四边形模式
                    # elif self.current_mode == "square": 
                    #     if self.is_drawing:
                    #         if not self.start_point:
                    #             self.start_point = (x, y)
                    #             self.x[0], self.y[0] = x, y
                    #             self.temp = self.scene.addEllipse(self.x[0]-2, self.y[0]-2, 4, 4,QPen(point_color),QBrush(point_color))
                    #             # self.labelInfo.setText(f"正四边形模式：已记录起点 ({x}, {y})，请点击终点")
                    #             self.drawonepoint = True
                    #         elif not self.end_point:
                    #             self.end_point = (x, y)
                    #             self.x[2], self.y[2] = x, y
                    #             self.x[1], self.y[1], self.x[3], self.y[3] = self.x[2], self.y[0], self.x[0], self.y[2]
                    #             self.cx = (self.x[0]+self.x[2])/2
                    #             self.cy = (self.y[0]+self.y[2])/2
                    #             self.R = math.sqrt((self.cx-self.x[0])**2+(self.cy-self.y[0])**2)
                    #             for i in range(4):
                    #                 self.cornor_angle[i] = math.atan2(self.y[i]-self.cy, self.x[i]-self.cx)
                    #             self.is_square_finalized = True
                    #             self.is_drawing = False
                    #             self.drawonepoint = False
                    #             # self.labelInfo.setText(f"正四边形模式：已记录终点 ({x}, {y})")
                    #             self.scene.removeItem(self.temp)
                    #             self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3],1)
                    #     elif self.is_square_finalized:
                    #         self.cornor_update()
                
        return super().eventFilter(source, event)

    # def paint_square(self, x1, y1, x2, y2, x3, y3, x4, y4, bold, type=1):
    #     self.del_paint_square()
    #     x0 = (x1 + x3)/2
    #     y0 = (y1 + y3)/2
    #     polygon = QPolygonF([
    #         QPointF(x1, y1),QPointF(x2, y2),
    #         QPointF(x3, y3),QPointF(x4, y4)
    #     ])
    #     poly_item = QGraphicsPolygonItem(polygon)
    #     self.current_polygon_item = poly_item
    #     # self.current_polygon_item = QGraphicsPolygonItem(QPolygonF([
    #     #     QPointF(x1, y1),QPointF(x2, y2),
    #     #     QPointF(x3, y3),QPointF(x4, y4)
    #     # ]))
    #     if type == 0:
    #         poly_item.setPen(QPen(line_color, 2*bold, Qt.DashLine))
    #     elif type == 1:
    #         poly_item.setPen(QPen(line_color, 2*bold))
    #     poly_item.setBrush(QBrush(QColor(200, 200, 255, 100))) # 半透明蓝色
    #     self.scene.addItem(poly_item)
    #     self.square_part_item.append(poly_item)
    #     self.point_ul = self.scene.addEllipse(x1-2*bold, y1-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
    #     self.point_ur = self.scene.addEllipse(x2-2*bold, y2-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
    #     self.point_dr = self.scene.addEllipse(x3-2*bold, y3-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
    #     self.point_dl = self.scene.addEllipse(x4-2*bold, y4-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
    #     self.point_ct = self.scene.addEllipse(x0-2*bold, y0-2*bold, 4*bold, 4*bold, QPen(point_color), QBrush(point_color))
    #     line1 = QGraphicsLineItem(x1, y1, x3, y3)
    #     line2 = QGraphicsLineItem(x2, y2, x4, y4)
    #     line1.setPen(QPen(line_color, 1*bold, Qt.DashLine))
    #     line2.setPen(QPen(line_color, 1*bold, Qt.DashLine))
    #     self.scene.addItem(line1)
    #     self.scene.addItem(line2)
    #     self.square_part_item += [line1, line2, self.point_ul, self.point_ur, self.point_dr, self.point_dl, self.point_ct]
    #     # self.labelInfo.setText(f"({x1:.3f}, {y1:.3f}),({x2:.3f}, {y2:.3f}),({x3:.3f}, {y3:.3f}),({x4:.3f}, {y4:.3f}),({x0:.3f},({y0:.3f}))")

    # def del_paint_square(self):
    #     for i in range(len(self.square_part_item)):
    #         self.scene.removeItem(self.square_part_item[i])
    #     self.square_part_item = []

    # def square_move(self, x, y, mode=1):
    #     if mode:
    #         dx = x - self.x0
    #         dy = y - self.y0
    #         self.x0, self.y0 = x, y
    #     else:
    #         dx, dy = x, y
    #     for i in range(4):
    #         self.x[i] += dx
    #         self.y[i] += dy
    #     self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 2)
    #     self.cornor_update()
    #     # self.labelInfo.setText(f"({self.x[0]:.3f}, {self.y[0]:.3f}),({self.x[1]:.3f}, {self.y[1]:.3f}),({self.x[2]:.3f}, {self.y[2]:.3f}),({self.x[3]:.3f}, {self.y[3]:.3f}),({self.cx:.3f}, {self.cy:.3f})")

    # def point_rotate(self, i): # i = 0,1,2,3
    #     angle_rad = math.radians(self.angle)+self.cornor_angle[i]
    #     x = self.R * math.cos(angle_rad)
    #     y = self.R * math.sin(angle_rad)
    #     totate_x = x + self.cx
    #     totate_y = y + self.cy
    #     return totate_x, totate_y

    # def square_rotate(self, angle_degrees): # angle为相对值，正顺负逆
    #     self.angle = self.angle + angle_degrees
    #     for i in range(4):
    #         self.x[i], self.y[i] = self.point_rotate(i)
    #     self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 2)

    # def square_stretch(self, i): # i = 0,1,2,3
    #     if i%2 == 0:
    #         self.x[1], self.y[1] = self.point_rotate(1)
    #         self.x[3], self.y[3] = self.point_rotate(3)
    #     else:
    #         self.x[0], self.y[0] = self.point_rotate(0)
    #         self.x[2], self.y[2] = self.point_rotate(2)
        
    #     self.paint_square(self.x[0], self.y[0], self.x[1], self.y[1], self.x[2], self.y[2], self.x[3], self.y[3], 2)

    # def cornor_update(self, i=1):
    #     if i == 1:
    #         self.cx = (self.x[0]+self.x[2])/2
    #         self.cy = (self.y[0]+self.y[2])/2
    #         self.R = math.sqrt((self.cx-self.x[0])**2+(self.cy-self.y[0])**2)
    #     else:
    #         self.cx = (self.x[1]+self.x[3])/2
    #         self.cy = (self.y[1]+self.y[3])/2
    #         self.R = math.sqrt((self.cx-self.x[1])**2+(self.cy-self.y[1])**2)
    #     # self.labelInfo.setText(f"({self.x[0]:.3f}, {self.y[0]:.3f}),({self.x[1]:.3f}, {self.y[1]:.3f}),({self.x[2]:.3f}, {self.y[2]:.3f}),({self.x[3]:.3f}, {self.y[3]:.3f}),({self.cx:.3f}, {self.cy:.3f})")

    def remove_temp_item(self):
        if self.temp_item:
            self.scene.removeItem(self.temp_item)
            self.temp_item = None

    def calculate_circle(self, p1, p2, p3):
        try:
            x1, y1 = map(float, p1)
            x2, y2 = map(float, p2)
            x3, y3 = map(float, p3)
            a = x2 - x1
            b = y2 - y1
            c = x3 - x1
            d = y3 - y1
            m1x, m1y = (x1 + x2)/2, (y1 + y2)/2
            m2x, m2y = (x1 + x3)/2, (y1 + y3)/2
            A1 = a
            B1 = b
            C1 = a*m1x + b*m1y
            A2 = c
            B2 = d
            C2 = c*m2x + d*m2y
            det = A1*B2 - A2*B1
            if abs(det) < 1e-6:
                return None, None, None
            h = (B2*C1 - B1*C2) / det
            k = (A1*C2 - A2*C1) / det
            radius = ((x1 - h)**2 + (y1 - k)**2)**0.5
            return h, k, radius
        except:
            return None, None, None
    
    def __del__(self):
        self.guiclient_node.destroy_node()
        self.encodersubscriber.destroy_node()
        self.camera_subscriber.destroy_node()
        rclpy.shutdown()

def main():
    app = QtWidgets.QApplication([])
    window = MainWindow()
    # window = MainWindow2(1)
    window.show()
    app.exec_()
    # rclpy.spin(node)    

if __name__ == "__main__":
    main()
