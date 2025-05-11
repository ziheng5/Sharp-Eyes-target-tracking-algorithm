from keyboard_control import *
from detection import *
from multiprocessing import Process


if __name__ == "__main__":
    # =========================================================
    # 调参
    depth_min_d = 0.5
    depth_max_d = 150
    # =========================================================

    process = [Process(target=keyboard_control, args=()),
               # Process(target=yolo_cv, args=()),
               # Process(target=dep_image, args=(depth_min_d, depth_max_d)),
               Process(target=yolo_and_depth, args=(depth_min_d, depth_max_d)),]

    [p.start() for p in process]
    [p.join() for p in process]