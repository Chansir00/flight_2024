import cv2
import numpy as np


stick_aspect_ratio = 1
def find_color_blocks_from_camera():
    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    while True:
        # 读取一帧图像
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头图像")
            break

        # 转换为HSV颜色空间
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 定义黑色色块的HSV范围
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])

        # 定义红色色块的HSV范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 创建黑色和红色的掩膜
        mask_black = cv2.inRange(hsv_frame, lower_black, upper_black)
        mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 找到黑色色块的轮廓
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_black:
            if cv2.contourArea(contour) > 500:  # 过滤掉太小的区域
                x, y, w, h = cv2.boundingRect(contour)
                if h/w > stick_aspect_ratio:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)

        # 找到红色色块的轮廓
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours_red:
            if cv2.contourArea(contour) > 500:  # 过滤掉太小的区域
                x, y, w, h = cv2.boundingRect(contour)
                if h/w > stick_aspect_ratio:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # 显示结果
        cv2.imshow("Detected Blocks", frame)

        # 按 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头和关闭窗口
    cap.release()
    cv2.destroyAllWindows()

# 示例用法
find_color_blocks_from_camera()
