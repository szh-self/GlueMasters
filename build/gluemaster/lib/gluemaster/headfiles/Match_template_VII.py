import cv2
import numpy as np
import time
from concurrent.futures import ThreadPoolExecutor
import os

# 主流程
def recognize_sign_pose(image, template):
    # 统一预处理
    image = preprocess_image(image)
    template = preprocess_image(template)  # 模板也需预处理

    # 特征提取（使用改进的边缘检测）
    image_feat = extract_robust_edges(image)
    template_feat = extract_robust_edges(template)

    # 提取模板ROI
    template_roi = extract_circular_roi(template)
    angle_range = (85, 100)
    # 金字塔旋转匹配
    best_angle, match_pos = optimized_pyramid_matching_rough(image, template_roi)
#    best_angle, match_pos = optimized_pyramid_matching_thin(image, template_roi, pyramid_levels=2, angle_range=angle_range)

    return best_angle, match_pos


#图像预处理（针对反光图像）
def preprocess_image(image,highlight_reflect=True):
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    #设置反光区域明度阈值
    lower_reflect = np.array([0,0,221],dtype=np.uint8)
    upper_reflect = np.array([180,30,255],dtype=np.uint8)
    mask = cv2.inRange(hsv,lower_reflect,upper_reflect)

    #对反光区域做自适应直方图均衡化
    image_yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    y_channel = image_yuv[:, :, 0]
    # CLAHE 实例
    clahe = cv2.createCLAHE(clipLimit=12.0, tileGridSize=(300, 300))
    y_equalized = clahe.apply(y_channel)

    # Step 3: 仅对反光区域的亮度进行替换
    image_yuv[:, :, 0] = np.where(mask == 255, y_equalized, y_channel)

    # Step 4: 转回 BGR
    result = cv2.cvtColor(image_yuv, cv2.COLOR_YUV2BGR)
    return result

#高斯模糊+Canny边缘检测
def extract_edges(image):
    blurred = cv2.GaussianBlur(image, (1,1), 0.5)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1=50, threshold2=100)
    return edges

#静态背景差分
def static_background_subtraction(foreground_img, background_img):
    # 尺寸对齐
    if foreground_img.shape != background_img.shape:
        background_img = cv2.resize(background_img, (foreground_img.shape[1], foreground_img.shape[0]))

    # 灰度化
    fg_gray = cv2.cvtColor(foreground_img, cv2.COLOR_BGR2GRAY)
    bg_gray = cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)

    # 差分取绝对值
    diff = cv2.absdiff(fg_gray, bg_gray)

    # 二值化
    _, fg_mask = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

    # 形态学操作去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_DILATE, kernel)

    # 使用掩码提取前景区域
    fg_result = cv2.bitwise_and(foreground_img, foreground_img, mask=fg_mask)
    return fg_result


#基于引导滤波的Canny边缘检测
def extract_robust_edges(image):
    """改进的边缘特征提取"""
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    # 使用引导滤波增强边缘
    guided = cv2.ximgproc.guidedFilter(
        guide=gray,
        src=gray,
        radius=5,
        eps=0.05
    )

    # 自适应Canny阈值
    median = np.median(guided)
    low_thresh = int(max(0, 0.7 * median))
    high_thresh = int(min(255, 1.3 * median))

    return cv2.Canny(guided, low_thresh, high_thresh)

#提取圆形ROI
def extract_circular_roi(image):
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    radius = min(center)  # 简单取最小边一半
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.circle(mask, center, radius, 255, -1)
    roi = cv2.bitwise_and(image, image, mask=mask)
    return roi

#金字塔旋转匹配算法
def rotate_image(img, angle):
    h, w = img.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    return cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR)


def match_template(main_img, template_img):
    method = cv2.TM_CCOEFF_NORMED
    result = cv2.matchTemplate(main_img, template_img, method)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    return max_val, max_loc

#搜索最优角度
def match_at_angle(angle, scaled_img, scaled_tpl):
    actual_angle = angle % 360
    rotated_tpl = rotate_image(scaled_tpl, actual_angle)
    result = cv2.matchTemplate(scaled_img, rotated_tpl, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    return (max_val, actual_angle, max_loc)

#匹配两次,一次粗匹配，一次精匹配
def optimized_pyramid_matching_rough(typelist ,image, templatelist, pyramid_levels=2):
    """优化后的金字塔匹配算法"""
    start_time = time.time()
    best_score = -1
    best_angle = 0
    best_pos = (0, 0)
    angle_range = (0, 360)  # 初始角度范围

    # 创建金字塔
    img_pyramid = [image]
    tpl_pyramid = [template]

    for i in range(pyramid_levels):
        img_pyramid.append(cv2.pyrDown(img_pyramid[-1]))
        tpl_pyramid.append(cv2.pyrDown(tpl_pyramid[-1]))

    # 从顶层开始向下匹配
    for level in range(pyramid_levels, -1, -1):
        scale = 2 ** level
        scaled_img = img_pyramid[level]
        scaled_tpl = tpl_pyramid[level]

        # 角度搜索范围随金字塔层级细化
        if level == pyramid_levels:  # 顶层使用粗搜索
            angle_step = 50
        elif level > 0:  # 中间层使用中等精度
            angle_step = 10
            angle_range = (best_angle - 45, best_angle + 45)  # 缩小范围
        else:  # 底层使用精细搜索
            angle_step = 1
            angle_range = (best_angle - 10, best_angle + 10)  # 进一步缩小范围

        # 构建角度列表（确保范围在0~359内）
#        angles = list(range(angle_range[0], angle_range[1] + 1, angle_step))
        angles = np.arange(angle_range[0], angle_range[1] + angle_step, angle_step)
        angles = [a % 360 for a in angles]

        max_workers = min(os.cpu_count() or 4, len(angles))  # 安全限制线程数

        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            results = executor.map(lambda a: match_at_angle(a, scaled_img, scaled_tpl), angles)
        # 当前层级的最佳结果
        level_best_score = -1
        level_best_angle = best_angle
        level_best_pos = best_pos

        # 角度搜索
        for max_val, actual_angle, max_loc in results:
            if max_val > level_best_score:
                level_best_score = max_val
                level_best_angle = actual_angle
                level_best_pos = max_loc

        # 更新全局最佳
        if level_best_score > best_score:
            best_score = level_best_score
            best_angle = level_best_angle
            best_pos = (level_best_pos[0] * scale, level_best_pos[1] * scale)

    print(f"匹配耗时：{time.time() - start_time:.3f}秒")
    return best_angle, best_pos

#第二次精匹配
def optimized_pyramid_matching_accurate(typelist,image, templatelist, pyramid_levels=2, angle_range=(0, 360)):
    start_time = time.time()
    best_score = -1
    best_angle = 0.0
    best_pos = (0, 0)

    img_pyramid = [image]
    tpl_pyramid = [templatelist[0]]
    for i in range(pyramid_levels):
        img_pyramid.append(cv2.pyrDown(img_pyramid[-1]))
        tpl_pyramid.append(cv2.pyrDown(tpl_pyramid[-1]))

    for level in range(pyramid_levels, -1, -1):
        scale = 2 ** level
        scaled_img = img_pyramid[level]
        scaled_tpl = tpl_pyramid[level]

        if level == pyramid_levels:
            search_range = angle_range
            angle_step = 4.40
        elif level > 0:
            search_range = (best_angle - 3.40, best_angle + 3.40)
            angle_step = 2.40
        else:
            search_range = (best_angle - 1.40, best_angle + 1.40)
            angle_step = 0.2  # 精度为0.2度

        angles = np.arange(search_range[0], search_range[1] + angle_step, angle_step)
        angles = [a % 360 for a in angles]

        max_workers = min(os.cpu_count() or 4, len(angles))

        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            results = executor.map(lambda a: match_at_angle(a, scaled_img, scaled_tpl), angles)

        level_best_score = -1
        level_best_angle = best_angle
        level_best_pos = best_pos

        for max_val, actual_angle, max_loc in results:
            if max_val > level_best_score:
                level_best_score = max_val
                level_best_angle = actual_angle
                level_best_pos = max_loc

        if level_best_score > best_score:
            best_score = level_best_score
            best_angle = level_best_angle
            best_pos = (level_best_pos[0] * scale, level_best_pos[1] * scale)

    print(f"匹配耗时：{time.time() - start_time:.3f}秒")
    print(f"最终匹配角度：{best_angle:.2f}°，位置：{best_pos}")
    return best_angle, best_pos


if __name__ == '__main__':
    image_path=r'G:\\embeded\\obb_datasets\\7ad3c52d386658448d466f426568f96.jpg'
    image_thin_path = r'C:\\Users\\Lenovo\\OneDrive\\Desktop\\WIN_20250605_21_52_30_Pro.jpg'
    template_path = r'G:\\embeded\\obb_datasets\\template\\template_5.jpg'
    template_thin_path = r'G:\\embeded\\obb_datasets\\template\\template_thin.jpg'
    img = cv2.imread(image_path)
    tpl = cv2.imread(template_path)
    angle, pos = recognize_sign_pose(img, tpl)
    print(f"识别角度：{angle}°，位置：{pos}")

    # 获取模板尺寸
    h, w = tpl.shape[:2]
    center = (float(pos[0] + w // 2), float(pos[1] + h // 2))

    # 定义旋转矩形（中心坐标，宽高，角度）
    rotated_rect = (center, (float(w), float(h)), float(-angle))  # OpenCV中顺时针为负
    box = cv2.boxPoints(rotated_rect)
    box = box.astype(int)

    # 绘制旋转矩形框
    boxed_img = img.copy()
    cv2.drawContours(boxed_img, [box], 0, (0, 255, 0), 2)

    # 显示并保存结果
    cv2.imshow("Matched with Rotated Box", boxed_img)
    cv2.imwrite("matched_result_5.jpg", boxed_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
