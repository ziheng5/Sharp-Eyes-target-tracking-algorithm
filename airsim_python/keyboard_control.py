import sys
import pygame
import airsim
import time


def keyboard_control():
    # pygame settings
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Keyboard Control")
    screen.fill((0, 0, 0))

    # airsim settings
    # 这里改为你要控制的无人机名称
    vehicle_name = "Drone1"
    AirSim_client = airsim.MultirotorClient()
    AirSim_client.confirmConnection()
    AirSim_client.enableApiControl(True, vehicle_name=vehicle_name)
    AirSim_client.armDisarm(True, vehicle_name=vehicle_name)
    AirSim_client.takeoffAsync(vehicle_name=vehicle_name).join()

    # 基础的控制速度(m/s)
    vehicle_velocity = 2.0
    # 设置临时加速度比例
    speedup_ratio = 10.0
    # 用来设置临时加速
    speedup_flag = False

    # 基础的偏航速率
    vehicle_yaw_rate = 5.0

    while True:

        yaw_rate = 0.0
        velocity_x = 0.0
        velocity_y = 0.0
        velocity_z = 0.0

        time.sleep(0.02)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        scan_wrapper = pygame.key.get_pressed()

        # 按下空格键加速 10 倍
        if scan_wrapper[pygame.K_SPACE]:
            scale_ratio = speedup_ratio
        else:
            scale_ratio = 1.0

        # 根据 "A" 和 "D" 按键来设置偏航速率变量
        if scan_wrapper[pygame.K_a] or scan_wrapper[pygame.K_d]:
            yaw_rate = (scan_wrapper[pygame.K_d] - scan_wrapper[pygame.K_a]) * scale_ratio * vehicle_yaw_rate

        # 根据 "UP" 和 "DOWN" 按键来设置 pitch 轴速度变量（NED 坐标系，x 为机头向前）
        if scan_wrapper[pygame.K_UP] or scan_wrapper[pygame.K_DOWN]:
            velocity_x = (scan_wrapper[pygame.K_UP] - scan_wrapper[pygame.K_DOWN]) * scale_ratio

        # 根据 "LEFT" 和 "RIGHT" 按键来设置 roll 轴速度变量（NED 坐标系，y 为正右方）
        if scan_wrapper[pygame.K_LEFT] or scan_wrapper[pygame.K_RIGHT]:
            velocity_y = (scan_wrapper[pygame.K_RIGHT] - scan_wrapper[pygame.K_LEFT]) * scale_ratio

        # 根据 "W" 和 "S" 按键来设置 z 轴速度变量（NED 坐标系，z 轴向上为负）
        if scan_wrapper[pygame.K_w] or scan_wrapper[pygame.K_s]:
            velocity_z = (scan_wrapper[pygame.K_s] - scan_wrapper[pygame.K_w]) * scale_ratio

        # 设置速度控制以及设置偏航控制
        AirSim_client.moveByVelocityBodyFrameAsync(vx=velocity_x, vy=velocity_y, vz=velocity_z, duration=0.02,
                                                   yaw_mode=airsim.YawMode(True, yaw_or_rate=yaw_rate), vehicle_name=vehicle_name)


        # press "Esc" to quit
        if scan_wrapper[pygame.K_ESCAPE]:
            AirSim_client.enableApiControl(False, vehicle_name=vehicle_name)
            AirSim_client.armDisarm(False, vehicle_name=vehicle_name)
            pygame.quit()
            sys.exit()