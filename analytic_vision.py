

"""
    Multi cameras sample showing how to open multiple ZED in one program
"""

import pyzed.sl as sl
import cv2
import numpy as np
import threading
import time
import signal
import sys
import ogl_viewer.viewer as gl
import pyzed.sl as sl
import matplotlib.pyplot as plt
import time

zed_list = []
left_list = []
depth_list = []
pointcloud_list = []
timestamp_list = []
thread_list = []
stop_signal = False

def signal_handler(signal, frame):
    global stop_signal
    stop_signal=True
    time.sleep(0.5)
    exit()

def grab_run(index, res):
    global stop_signal
    global zed_list
    global timestamp_list
    global left_list
    global depth_list
    global pointcloud_list

    runtime = sl.RuntimeParameters()
    while not stop_signal:
        err = zed_list[index].grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
            zed_list[index].retrieve_measure(depth_list[index], sl.MEASURE.DEPTH)
            zed_list[index].retrieve_measure(pointcloud_list[index], sl.MEASURE.XYZRGBA, sl.MEM.CPU, res)
            timestamp_list[index] = zed_list[index].get_timestamp(sl.TIME_REFERENCE.CURRENT).data_ns
        time.sleep(0.001) #1ms
    zed_list[index].close()
	
def main():
    global stop_signal
    global zed_list
    global left_list
    global depth_list
    global timestamp_list
    global thread_list
    global pointcloud_list
    signal.signal(signal.SIGINT, signal_handler)

    print("Running...")
    # init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
    #                              coordinate_units=sl.UNIT.METER,
    #                              coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
    init = sl.InitParameters()

    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30  # The framerate is lowered to avoid any USB3 bandwidth issues
    res = sl.Resolution()
    res.width = 720
    res.height = 404
    
    #List and open cameras
    name_list = []
    last_ts_list = []
    cameras = sl.Camera.get_device_list()
    index = 0
    for cam in cameras:
        init.set_from_serial_number(cam.serial_number)
        name_list.append("ZED {}".format(cam.serial_number))
        print("Opening {}".format(name_list[index]))
        zed_list.append(sl.Camera())
        left_list.append(sl.Mat())
        depth_list.append(sl.Mat())
        pointcloud_list.append(sl.Mat(res.width, res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU))
        timestamp_list.append(0)
        last_ts_list.append(0)
        status = zed_list[index].open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()
        index = index +1

    #Start camera threads
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            thread_list.append(threading.Thread(target=grab_run, args=(index,res,)))
            thread_list[index].start()

    camera_model = zed_list[0].get_camera_information().camera_model
    # # # Create OpenGL viewer
    # viewer = gl.GLViewer()
    # viewer.init(1, sys.argv, camera_model, res)

    time.sleep(1.0)
    #Display camera images
    key = ''
    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            if (timestamp_list[index] > last_ts_list[index]):
                breakpoint()
                depth_img = depth_list[index].get_data()
                # REMINDER : 12.7mm for half an inch!!!!
                # set values in the image that are nan to 0 so we can display it 
                depth_img = np.where(np.isnan(depth_img) | np.isinf(depth_img), 0, depth_img)
                depth_img[np.isnan(depth_img)]=0
                plt.imshow(depth_img, interpolation='nearest')
                plt.show()
                # cv2.imshow(name_list[index], depth_list[index].get_data())
                x = round(depth_list[index].get_width() / 2)
                y = round(depth_list[index].get_height() / 2)
                err, depth_value = depth_list[index].get_value(x, y)
                if np.isfinite(depth_value):
                    print("{} depth at center: {}MM".format(name_list[index], round(depth_value)))
                last_ts_list[index] = timestamp_list[index]
        key = cv2.waitKey(10)
        # cv2.destroyAllWindows()

    #Stop the threads
    stop_signal = True
    for index in range(0, len(thread_list)):
        thread_list[index].join()

    print("\nFINISH")

if __name__ == "__main__":
    main()
