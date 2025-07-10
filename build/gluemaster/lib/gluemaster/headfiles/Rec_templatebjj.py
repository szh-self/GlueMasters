import cv2
import numpy as np
#此文件未采用
# 加载图像
template = cv2.imread('\\0293e983f8515542e8f4d1becc430bd.jpg')
target = cv2.imread('\\2d8a462b5a02f8373cbb52b94aa2ee0.png')

# 预处理：转灰度、阈值处理
gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
gray_target = cv2.cvtColor(target, cv2.COLOR_BGR2GRAY)
_, thresh_template = cv2.threshold(gray_template, 127, 255, cv2.THRESH_BINARY)
_, thresh_target = cv2.threshold(gray_target, 127, 255, cv2.THRESH_BINARY)

# 提取模板轮廓
contours_template, _ = cv2.findContours(thresh_template, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_L1)
template_contour = max(contours_template, key=cv2.contourArea)

# 提取目标图像轮廓
contours_target, _ = cv2.findContours(thresh_target, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_L1)

# 轮廓匹配
matches = []
MATCH_THRESHOLD = 0.5
for cnt in contours_target:
    similarity = cv2.matchShapes(template_contour, cnt, cv2.CONTOURS_MATCH_I2, 0)
    if similarity < MATCH_THRESHOLD:
        matches.append(cnt)

# 绘制结果
result = cv2.cvtColor(gray_target, cv2.COLOR_GRAY2BGR)
cv2.drawContours(result, matches, -1, (0, 255, 0), 2)

cv2.imshow('Result', result)
cv2.waitKey(0)
cv2.destroyAllWindows()