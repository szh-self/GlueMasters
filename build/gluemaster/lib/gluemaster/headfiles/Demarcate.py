import cv2
import numpy as np


#棋盘格角点检测
def detect_and_count_chessboard_corners(image):

    if image is None:
        raise FileNotFoundError(f"无法加载图像")

    # 处理图像通道
    if len(image.shape) == 3 and image.shape[2] == 4:
        alpha = image[:, :, 3]
        bg_color = [255, 255, 255]
        image_rgb = image[:, :, :3].copy()
        mask = alpha == 0
        for c in range(3):
            image_rgb[mask, c] = bg_color[c]
        image = image_rgb
    elif len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        image = image[:, :, :3]  # 确保只有RGB通道

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 图像预处理
    gray = cv2.equalizeHist(gray)  # 增强对比度
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # 减少噪声

    # 尝试不同的棋盘格尺寸 - 注意：这些是内部角点的尺寸（行数-1, 列数-1）
    pattern_sizes = [(10, 7), (7, 4), (9,6), (6, 3), (8, 6), (8, 5), (11, 8)]

    corners_found = False
    corners = None
    pattern_size = None
    actual_pattern = None

    # 尝试预设尺寸 - 确保尺寸大于2x2
    for rows, cols in pattern_sizes:
        # 检查尺寸是否有效
        if cols < 3 or rows < 3:
            continue

        try:
            ret, detected_corners = cv2.findChessboardCorners(
                gray, (cols, rows),  # 注意：这里是 (列数, 行数)
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                    cv2.CALIB_CB_NORMALIZE_IMAGE +
                    cv2.CALIB_CB_FAST_CHECK
            )
            if ret:
                corners_found = True
                corners = detected_corners
                pattern_size = (rows, cols)
                print(f"使用预设尺寸检测成功: 内部角点: {rows}x{cols}")
                break
        except cv2.error as e:
            print(f"尺寸 {rows}x{cols} 检测失败: {e}")

    # 尝试自动检测尺寸 - 确保尺寸大于2x2
    if not corners_found:
        print("常见尺寸未检测到，尝试自动检测...")
        for rows in range(3, 15):  # 从3开始，确保大于2
            for cols in range(3, 15):  # 从3开始，确保大于2
                # 跳过1:1比例，因为棋盘格通常不是正方形
                if abs(rows - cols) < 2:
                    continue

                try:
                    ret, detected_corners = cv2.findChessboardCorners(
                        gray, (cols, rows),
                        flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                            cv2.CALIB_CB_NORMALIZE_IMAGE
                    )
                    if ret:
                        corners_found = True
                        corners = detected_corners
                        pattern_size = (cols, rows)
                        actual_pattern = (rows + 1, cols + 1)  # 实际方格尺寸
                        print(f"自动检测到尺寸: {cols + 1}x{rows + 1} 方格 (内部角点: {cols}x{rows})")
                        break
                except cv2.error as e:
                    print(f"尺寸 {cols}x{rows} 检测失败: {e}")
            if corners_found:
                break

    if not corners_found:
        print("未检测到棋盘格角点！请尝试以下建议：")
        print("1. 确保棋盘格在图像中清晰可见")
        print("2. 调整图像角度使棋盘格正面朝向")
        print("3. 检查光照条件是否均匀")
        print("4. 尝试更高分辨率的图像")
        return 0, 0  # 返回默认值

    # 提高角点检测精度
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    image_with_corners = cv2.drawChessboardCorners(image.copy(), pattern_size, corners, True)
    cv2.imshow("image_with_corner", image_with_corners)
    cv2.namedWindow("image_with_corner", cv2.WINDOW_AUTOSIZE)
    cv2.waitKey(10000)  # 显示3秒
    cv2.destroyAllWindows()
    # 返回内部角点尺寸
    return pattern_size[0], pattern_size[1]


# 计算去畸变后x，y轴的坐标转换参数
#frames:frame list; square_size_cm：每格多少厘米  mtx：相机内参 dist：畸变参数
def calculate_pixel_per_mm(frames, chessboard_size=None, square_size_cm=None, mtx=np.array([[3740.38684, 0, 311.070638],[0, 3751.82897, 240.049690],[0, 0, 1]]), dist=np.array([[-10.1169162, 664.415387, 0.0705176787, -0.00684608455, -30470.9261]])):
    square_size_mm = square_size_cm * 10
    pixel_per_mm_x_list = []
    pixel_per_mm_y_list = []

    # 如果未提供棋盘格尺寸，则从第一张图片中检测
    if chessboard_size is None:
        try:
            x_points, y_points = detect_and_count_chessboard_corners(frames[0])
            chessboard_size = (y_points, x_points)  # OpenCV使用(列数, 行数)
            print(f"检测到的棋盘格尺寸: {x_points}×{y_points} (内部角点)")
        except Exception as e:
            print(f"检测棋盘格尺寸时出错: {e}")
            return None, None

    for img in frames:
        if img is None:
            print(f"无法读取图像")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if not ret:
            print(f"在图像中未找到棋盘格角点")
            continue

        # 去畸变处理
        undistorted = cv2.undistortPoints(corners, mtx, dist, P=mtx)
        undistorted = undistorted.reshape(-1, 2)

        # 计算x轴方向
        pt1_x = undistorted[0]
        pt2_x = undistorted[1]
        dist_pixel_x = np.linalg.norm(pt2_x - pt1_x)

        # 计算y轴方向
        pt1_y = undistorted[0]
        pt2_y = undistorted[chessboard_size[0]]  # 跨一行
        dist_pixel_y = np.linalg.norm(pt2_y - pt1_y)

        pixel_per_mm_x_list.append(dist_pixel_x / square_size_mm)
        pixel_per_mm_y_list.append(dist_pixel_y / square_size_mm)

    if pixel_per_mm_x_list and pixel_per_mm_y_list:
        return (
            np.mean(pixel_per_mm_x_list),
            np.mean(pixel_per_mm_y_list)
        )
    else:
        print("没有成功处理任何图像")
        return None, None