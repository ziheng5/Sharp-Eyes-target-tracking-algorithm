from ultralytics import YOLO
import numpy as np
import airsim
import time
import cv2


def yolo_cv():
    # 初始化 YOLOv8 模型
    model = YOLO('yolov8s.pt')

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # 设置图像类型（可以是Scene, Depth, Segmentation等）
    image_type = airsim.ImageType.Scene

    # 设置摄像头名称
    camera_name = "front_center"

    # 设置显示窗口
    cv2.namedWindow("Drone FPV View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone FPV View", 1000, 600)

    # FPS 计算变量
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # 获取前端摄像头图像
            responses = client.simGetImages([
                # 不返回浮点数，不压缩
                airsim.ImageRequest(camera_name=camera_name, image_type=image_type,
                                    pixels_as_float=False, compress=False),
            ])

            response = responses[0]

            # 处理场景图像
            ## 将图像数据转为 numpy 数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            ## 重塑数组为 3 通道图像
            frame = img1d.reshape(response.height, response.width, 3)

            ## 目标检测
            results = model.predict(frame, classes=[2])

            for result in results:
                annotated_frame = result.plot()

                # 计算并显示FPS
                frame_count += 1
                if frame_count >= 30:  # 每30帧计算一次FPS
                    fps = frame_count / (time.time() - start_time)
                    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    frame_count = 0
                    start_time = time.time()

                # 显示图像
                cv2.imshow("Drone FPV View", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        cv2.destroyAllWindows()


def dep_image(min_d, max_d):
    AirSim_client = airsim.MultirotorClient()
    AirSim_client.confirmConnection()
    AirSim_client.enableApiControl(True)
    AirSim_client.armDisarm(True)

    # 设置图像类型（可以是Scene, Depth, Segmentation等）
    image_type = airsim.ImageType.DepthPerspective

    # 设置摄像头名称
    camera_name = "front_center"

    # 设置显示窗口
    cv2.namedWindow("Drone FPV DEPTH View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone FPV DEPTH View", 1000, 600)

    try:
        while True:
            responses = AirSim_client.simGetImages([
                airsim.ImageRequest(camera_name=camera_name, image_type=airsim.ImageType.DepthPerspective,
                                    pixels_as_float=True, compress=False)
            ])

            depth_response = responses[0]

            # 1. 处理深度图像
            depth_data = airsim.list_to_2d_float_array(depth_response.image_data_float,
                                                       depth_response.width, depth_response.height)
            # 截断有效范围（min_d ~ max_d）
            valid_depth = np.clip(depth_data, min_d, max_d)  # 小于 min_d 设为 min_d，大于 max_d 设为 max_d

            # 归一化
            depth_normalized = (255- (valid_depth - min_d) / (max_d - min_d) * 255).astype(np.uint8)

            # 2. **应用颜色映射（可选）**
            # 方案1：黑白灰度图（推荐）
            depth_visualized = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            cv2.imshow("Drone FPV DEPTH View", depth_visualized)

            if cv2.waitKey(1) & 0xFF == ord('e'):
                break

    finally:
        AirSim_client.armDisarm(False)
        AirSim_client.enableApiControl(False)


def yolo_and_depth(min_d, max_d):
    # 初始化 YOLOv8 模型
    model = YOLO('yolov8s.pt')

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # 设置图像类型（可以是Scene, Depth, Segmentation等）
    image_type = airsim.ImageType.Scene

    # 设置摄像头名称
    camera_name = "front_center"

    # 设置显示窗口 1
    cv2.namedWindow("Drone FPV View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone FPV View", 1000, 600)

    # 设置显示窗口 2
    cv2.namedWindow("Drone FPV DEPTH View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Drone FPV DEPTH View", 1000, 600)

    # FPS 计算变量
    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # 获取前端摄像头图像
            responses = client.simGetImages([
                # 不返回浮点数，不压缩
                airsim.ImageRequest(camera_name=camera_name, image_type=image_type,
                                    pixels_as_float=False, compress=False),
                airsim.ImageRequest(camera_name=camera_name, image_type=airsim.ImageType.DepthPerspective,
                                    pixels_as_float=True, compress=False),
            ])

            response = responses[0]
            depth_response = responses[1]

            # 1. 处理深度图像
            depth_data = airsim.list_to_2d_float_array(depth_response.image_data_float,
                                                       depth_response.width, depth_response.height)
            # 截断有效范围（min_d ~ max_d）
            valid_depth = np.clip(depth_data, min_d, max_d)  # 小于 min_d 设为 min_d，大于 max_d 设为 max_d

            # 归一化
            depth_normalized = (255 - (valid_depth - min_d) / (max_d - min_d) * 255).astype(np.uint8)

            # 绘制黑白灰度图
            depth_visualized = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)

            # 显示深度图
            cv2.imshow("Drone FPV DEPTH View", depth_visualized)

            # 2. 处理场景图像
            ## 将图像数据转为 numpy 数组
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)

            ## 重塑数组为 3 通道图像
            frame = img1d.reshape(response.height, response.width, 3)

            ## 目标检测
            results = model.predict(frame, classes=[2])

            for result in results:
                annotated_frame = result.plot()

                # 计算并显示FPS
                frame_count += 1
                if frame_count >= 30:  # 每30帧计算一次FPS
                    fps = frame_count / (time.time() - start_time)
                    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    frame_count = 0
                    start_time = time.time()

                # 显示图像
                cv2.imshow("Drone FPV View", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        cv2.destroyAllWindows()