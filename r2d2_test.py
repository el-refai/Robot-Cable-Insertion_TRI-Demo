
import numpy as np
import time
import pyzed.sl as sl
import ogl_viewer.viewer as gl
import math
import sys

def main():
    varied_camera_1_id = 22008760  # Left camera
    varied_camera_2_id = 24400334  # Right camera
    calibration_matrices = {
        f'{varied_camera_1_id}'+"_left": np.array([ 0.24653256,  0.59249144,  0.28827498, -1.9806605 , -0.00783103,
       -2.47334296]),
       f'{varied_camera_1_id}'+"_right":np.array([ 0.15232565,  0.51752746,  0.28852659, -1.97916377, -0.0084648 ,
       -2.46817059]),
       f'{varied_camera_2_id}'+"_left":np.array([ 0.13973208, -0.40084596,  0.30084572, -1.99526056,  0.05251052,
       -1.04160147]),
       f'{varied_camera_2_id}'+"_right":np.array([ 0.19866192, -0.50710121,  0.2967499 , -2.00089156,  0.05840376,
       -1.04475616])
    }
    # robot =  ServerInterface(ip_address=nuc_ip)

    # Create a Camera object
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 15  # The framerate is lowered to avoid any USB3 bandwidth issues

    #List and open cameras
    cam_ids = [varied_camera_1_id, varied_camera_2_id]
    name_list = []
    zed_list = []
    left_list = []
    pointcloud_list = []
    index =0


    for cam_id in cam_ids:
        init.set_from_serial_number(cam_id)
        name_list.append("ZED {}".format(cam_id))
        print("Opening {}".format(name_list[index]))
        zed_list.append(sl.Camera())
        left_list.append(sl.Mat())
        pointcloud_list.append(sl.Mat())
        status = zed_list[index].open(init)
        breakpoint()
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            zed_list[index].close()
        index = index +1

    for index in range(0, len(zed_list)):
        if zed_list[index].is_opened():
            runtime_parameters = sl.RuntimeParameters()
            camera_model = zed_list[index].get_camera_information().camera_model
            res = sl.Resolution()
            res.width = 720
            res.height = 404

            if zed_list[index].grab() == sl.ERROR_CODE.SUCCESS:
                zed_list[index].retrieve_image(left_list[index], sl.VIEW.LEFT)
                zed_list[index].retrieve_measure(pointcloud_list[index], sl.MEASURE.XYZRGBA)
    breakpoint()
                





    

            






#     side_cam = sl.Camera()
#     # Create a InitParameters object and set configuration parameters
#     init_params = sl.InitParameters()
#     init_params.set_from_serial_number(22008760)
#     init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
#     init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
#     init_params.camera_resolution = sl.RESOLUTION.HD720

#     # Open the camera
#     status = side_cam.open(init_params)
#     if status != sl.ERROR_CODE.SUCCESS:
#         raise RuntimeError('Camera Failed To Open')
#     if f"{side_cam.get_camera_information().serial_number}" == varied_camera_1_id:
#         # Create and set RuntimeParameters after opening the camera
#         runtime_parameters = sl.RuntimeParameters()
#         runtime_parameters.confidence_threshold = 100
#         runtime_parameters.texture_confidence_threshold = 100

#         camera_model = side_cam.get_camera_information().camera_model
#         res = sl.Resolution()
#         res.width = 720
#         res.height = 404

#         # Capture 150 images and depth, then stop
#         _left_img = sl.Mat()
#         _right_img = sl.Mat()
#         _sbs_img = sl.Mat()
#         _left_depth = sl.Mat()
#         _right_depth = sl.Mat()
#         _left_pointcloud = sl.Mat()
#         _right_pointcloud = sl.Mat()


#         # mirror_ref = sl.Transform()
#         # mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
#         # tr_np = mirror_ref.m
#         viewer = gl.GLViewer()
#         viewer.init(1,sys.argv, camera_model, res)

#         while viewer.is_available():
#             # A new image is available if grab() returns SUCCESS
#             if side_cam.grab() == sl.ERROR_CODE.SUCCESS:

#                 # Retrieve left image
#                 side_cam.retrieve_image(_left_img, sl.VIEW.LEFT)
#                 # side_cam.retrieve_image(_right_img, sl.VIEW.RIGHT)
#                 # side_cam.retrieve_image(_sbs_img, sl.VIEW.SIDE_BY_SIDE)
#                 # # Retrieve depth map. Depth is aligned on the left image
#                 # side_cam.retrieve_measure(_left_depth, sl.MEASURE.DEPTH)
#                 # side_cam.retrieve_measure(_right_depth, sl.MEASURE.DEPTH_RIGHT)
#                 # Retrieve colored point cloud. Point cloud is aligned on the left image.
#                 side_cam.retrieve_measure(_left_pointcloud, sl.MEASURE.XYZRGBA)
#                 # side_cam.retrieve_measure(_right_pointcloud, sl.MEASURE.XYZRGBA_RIGHT)
#                 # Get and print distance value in mm at the center of the image
#                 # We measure the distance camera - object using Euclidean distance

#                 viewer.updateData(_left_pointcloud)

#                 x = round(_left_img.get_width() / 2)
#                 y = round(_left_img.get_height() / 2)
#                 err, point_cloud_value = _left_pointcloud.get_value(x, y)
#                 x,y,z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
#                 print(x,y,z)
#                 print()
#                 distance = math.sqrt(x*x +y*y+z*z)
#                 if(viewer.save_data == True):
#                     point_cloud_to_save = sl.Mat();
#                     side_cam.retrieve_measure(_left_pointcloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
#                     err = point_cloud_to_save.write('Pointcloud.ply')
#                     if(err == sl.ERROR_CODE.SUCCESS):
#                         print("point cloud saved")
#                     else:
#                         print("the point cloud has not been saved")
#                     viewer.save_data = False

#                 #update robot:
#                 # robot.update_pose(np.array([0.5058340430259705, 0.4134461283683777, 0.31671109795570374, -2.6820327400280735, 0.006462129940462802, 0.14385363878820034]),velocity=False, blocking=True)

#                 # point_cloud_np = point_cloud.get_data()
#                 # point_cloud_np.dot(tr_np)

#                 if not np.isnan(distance) and not np.isinf(distance):
#                     print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
#                     # Increment the loop
#                 else:
#                     print("Can't estimate distance at this position.")
#                     print("Your camera is probably too close to the scene, please move it backwards.\n")
#                 sys.stdout.flush()

#         # Close the camera
#         viewer.exit()
#         side_cam.close()

if __name__ == "__main__":
    main()






# # robot =  ServerInterface(ip_address=nuc_ip)
# # # while True:
# #     # TODO effect of force?
# # # if we want to grasp tightly, we call grasp once. Other than velocity, no parameters matter. Alternatively, this is equivalent to use velocity=True in goto.
# # # if we want to go to a width, we need to use a while loop. velocity=False. absolute width is between 0 and 0.085. force doesn't matter?
# #     # robot.update_gripper_analytic(0.95,velocity=True, blocking=True, force=0.01)
# # current_joints=robot.get_joint_positions()
# # print(current_joints)
# # # robot.update_joints(np.array([0.1,0.3,0.3,0.3,0.3,0.3,0.3]), velocity=True)
# # # robot.update_joints(np.array([ 0.24807443,0.4416678,0.48023671 ,-1.84507334, 0.35278192,2.49715495,
# # # 0.28128988]), velocity=False, blocking=True)
# # robot.update_pose(np.array([0.5058340430259705, 0.4134461283683777, 0.31671109795570374, -2.6820327400280735, 0.006462129940462802, 0.14385363878820034]),velocity=False, blocking=True)
# # time.sleep(0.3)
# # current_joints=robot.get_joint_positions()
# # print(current_joints)
# # time.sleep(0.5)
# # current_pose = robot.get_ee_pose()
# # robot_state = robot.get_robot_state()
# # print("ee_pose:",current_pose)
# # print(robot_state)
