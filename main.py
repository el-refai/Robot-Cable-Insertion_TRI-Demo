import cv2
from scipy.ndimage.filters import gaussian_filter
from scipy.optimize import curve_fit
# from grasp import Grasp, GraspSelector
from autolab_core import RigidTransform, RgbdImage, DepthImage, ColorImage, CameraIntrinsics, Point, PointCloud
import numpy as np
import math
import copy
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
# from rotate import rotate_from_pointcloud, rotate
import time
import os
import sys
import traceback
from skimage.morphology import skeletonize
from skimage import data
import matplotlib.pyplot as plt
from skimage.util import invert
import pdb


from utils import *
from depth_sensing import get_rgb_get_depth

DISPLAY = True
original_cable_waypoints = None
num_iterations = 0

while True:
    depth_image, rgb_image = get_rgb_get_depth()
    g = GraspSegmenter(depth_image, rgb_image)
    plt.imshow(rgb_image, interpolation="nearest")
    plt.show()
    three_mat_color = rgb_image
    three_mat_depth = depth_image

    edges_pre = np.uint8(three_mat_depth*10)
    edges = cv2.Canny(edges_pre,10,20)
    plt.imshow(edges)
    plt.show()

     # ----------------------FIND END OF CHANNEL
    lower = 254
    upper = 256
    channel_start = (0, 0)
    max_edges = 0
    candidate_channel_pts = []
    
    # guess for what 0.5in is in terms of depth
    depth_diff_goal = 0.016
    # threshold to allow for error
    depth_threshold = 0.002

    for r in range(len(edges)):
            for c in range(len(edges[r])):
                if (lower < edges[r][c]< upper):
                    diff1 = 0
                    diff2 = 0
                    diff3 = 0
                    diff4 = 0
                    for add in range(1, 4):
                        if (r-add < 0 or c-add < 0) or (r+add >= len(three_mat_depth) or c+add >= len(three_mat_depth[r])):
                            break
                        # top - bottom
                        diff1 = abs(three_mat_depth[r-add][c] - three_mat_depth[r+add][c]) 
                        # left - right
                        diff2 = abs(three_mat_depth[r][c-add] - three_mat_depth[r][c+add])
                        # top left - bottom right
                        diff3 = abs(three_mat_depth[r-add][c-add] - three_mat_depth[r+add][r+add])
                        # top right - bottom left
                        diff4 = abs(three_mat_depth[r-add][c+add] - three_mat_depth[r+add][r-add])

                        if diff1 > 0.03 or diff2 > 0.03 or diff3 > 0.03 or diff4 > 0.03:
                            continue
                        if 0.01 <= np.mean(np.array([diff1, diff2, diff3, diff4])) <= 0.014:
                            candidate_channel_pts += [(r,c)]

                        # if 0.01 < diff1 < 0.014 or 0.01 < diff2 < 0.014 or 0.01 < diff3 < 0.014 or 0.01 < diff4 < 0.014:
                        #     candidate_channel_pts += [(r,c)]     
                    # throw away values that we know differ by too much, this is cause if you take the avg of diffs 
                    # if diff1 > 0.02:
                    #     diff1 = 0
                    # if diff2 > 0.02:
                    #     diff2 = 0
                    # if diff3 > 0.02:
                    #     diff3 = 0
                    # if diff4 > 0.02:
                    #     diff4 = 0 
                    if diff1 > 0.02 or diff2 > 0.02 or diff3 > 0.02 or diff4 > 0.02:
                        continue
                    if 0.01 <= np.mean(np.array([diff1, diff2, diff3, diff4])) <= 0.014:
                        candidate_channel_pts += [(r,c)]
                        #print("the detected avg was: ", np.mean(np.array([diff1, diff2, diff3, diff4])))
    print("Candidate Edge pts: ", candidate_channel_pts)
    # need to figure out which edge point is in fact the best one for our channel
    # i.e. highest up, and pick a point that is actually in the channel
    max_depth = 100000
    min_depth = 0
    channel_edge_pt = (0,0)
    channel_start = (0,0)
    sorted_candidate_channel_pts = sorted(candidate_channel_pts, key=lambda x: three_mat_depth[x[0]][x[1]])
    print("The sorted list is: ", sorted_candidate_channel_pts)
    possible_cable_edge_pt = sorted_candidate_channel_pts[-1]
    print("the edge with deepest depth is: ", three_mat_depth[possible_cable_edge_pt[0]][possible_cable_edge_pt[1]])

    for candidate_pt in candidate_channel_pts:
            r = candidate_pt[0]
            c = candidate_pt[1]
            print("r", r, "c", c, "my depth is: ", three_mat_depth[r][c])
            if 0 < three_mat_depth[r][c] < max_depth:
                print("max depth:", max_depth)
                channel_edge_pt = (r,c)
                max_depth = three_mat_depth[r][c]
            if three_mat_depth[r][c] > min_depth:
                possible_cable_edge_pt = (r,c)
                min_depth = three_mat_depth[r][c]
    print("The edge of the channel is: ", channel_edge_pt)
    r,c = channel_edge_pt
    possible_channel_pts = []

    ##### NEED TO REMOVE THE EDGES OF VALUE 0 FROM THE SAMPLE BASE!!!!!
    index = 0
    while index < len(sorted_candidate_channel_pts) and channel_start == (0,0):
        channel_edge_pt = sorted_candidate_channel_pts[index]
        r,c = channel_edge_pt
        if three_mat_depth[r][c] == 0.0:
            index += 1
            continue
        for add in range(1, 4):
            if (r-add < 0 or c-add < 0) or (r+add >= len(three_mat_depth) or c+add >= len(three_mat_depth[r])):
                break
            # left - right
            diff1 = abs(three_mat_depth[r-add][c] - three_mat_depth[r+add][c])
            diff2 = abs(three_mat_depth[r][c-add] - three_mat_depth[r][c+add])
            if 0.01 <= diff1 < 0.014: # prev upper was 0.016
                if three_mat_depth[r-add][c] > three_mat_depth[r+add][c]:
                    channel_start = (r-add, c)
                    possible_channel_pts += [(r-add, c)]
                else:
                    channel_start = (r+add, c)
                    possible_channel_pts += [(r+add, c)]
            if 0.01 <= diff2 < 0.014: #prev upper was 0.016
                if three_mat_depth[r][c-add] > three_mat_depth[r][c+add]:
                    channel_start = (r, c-add)
                    possible_channel_pts += [(r, c-add)]
                else:
                    channel_start = (r, c+add)
                    possible_channel_pts += [(r, c+add)]
        # the point in the channel was not found, so we need to look at the next best one
        if channel_start == (0,0):
            index += 1
    # channel_start = (channel_edge_pt[1], channel_edge_pt[0])
    print("possible channel pts: ", possible_channel_pts)
    print("The chosen channel_pt is: ", channel_start)

    print("CHANNEL_START: "+str(channel_start))
        
    # FINDING THE POINT ON THE CABLE!!!
    r = possible_cable_edge_pt[0]
    c = possible_cable_edge_pt[1]
    index = 0
    cable_pt = (0,0)
    while index < len(sorted_candidate_channel_pts) and cable_pt == (0,0):
        possible_cable_pt = sorted_candidate_channel_pts[-index]
        r,c = possible_cable_pt
        if three_mat_depth[r][c] == 0.0:
            index += 1
            continue
        for add in range(1, 8):
            # once we've found a suitable cable point we want to just exit
            if cable_pt != (0,0):
                break
            if (r-add < 0 or c-add < 0) or (r+add >= len(three_mat_depth) or c+add >= len(three_mat_depth[r])):
                break
            # left - right
            diff1 = abs(three_mat_depth[r-add][c] - three_mat_depth[r+add][c])
            diff2 = abs(three_mat_depth[r][c-add] - three_mat_depth[r][c+add])
            if 0.01 <= diff1 < 0.020: # prev upper was 0.016
                # the depth that is lower (i.e. point is closer to the camera is the point that is actually on the cable)
                if three_mat_depth[r-add][c] > three_mat_depth[r+add][c]:
                    cable_pt = (r+add, c)
                else:
                    cable_pt = (r-add,c)
            if 0.01 <= diff2 < 0.020: #prev upper was 0.016
                if three_mat_depth[r][c-add] > three_mat_depth[r][c+add]:
                    cable_pt = (r,c+add)
                else:
                    cable_pt = (r,c-add)
    # the point in the cable was not found, so we need to look at the next best one
        if cable_pt == (0,0):
            index += 1
    
    loc = (cable_pt[1], cable_pt[0])
    max_scoring_loc = loc
    print("the cable point is ", cable_pt)


    plt.imshow(edges, cmap='gray')
    plt.scatter(x = [j[1] for j in candidate_channel_pts], y=[i[0] for i in candidate_channel_pts],c='r')
    plt.scatter(x=channel_edge_pt[1], y=channel_edge_pt[0], c='b')
    plt.scatter(x=channel_start[1], y=channel_start[0], c='m')
    plt.scatter(x=cable_pt[1], y=cable_pt[0], c='w')
    plt.imshow(three_mat_depth, interpolation="nearest")
    plt.show()


    print("Starting segment_cable pt: "+str(max_scoring_loc))
    # ----------------------Segment
    rope_cloud, _, cable_waypoints, weird_pts = g.segment_cable(loc)
    # ----------------------




    fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(8, 4),
                        sharex=True, sharey=True)

    ax = axes.ravel()
    plt.scatter(x = [j[1] for j in weird_pts], y=[i[0] for i in weird_pts],c='w')
    plt.scatter(x = [j[1] for j in rope_cloud], y=[i[0] for i in rope_cloud],c='r')
    ax[0].imshow(three_mat_color, cmap=plt.cm.gray)
    ax[0].axis('off')
    ax[0].set_title('color', fontsize=20)

    ax[1].imshow(edges, cmap=plt.cm.gray)
    ax[1].axis('off')
    ax[1].set_title('edge', fontsize=20)

    fig.tight_layout()
    plt.show()

    blank = np.zeros((len(three_mat_depth[0]), len(three_mat_depth)))
    # for every point in the ropecloud we set it to 1 so we have our cable segment
    for r,c in rope_cloud:
        blank[r][c] = 1
    cable_segment_mask = blank
    plt.title("cable_segment")
    plt.imshow(cable_segment_mask)
    plt.show()
    cable_skeleton = skeletonize_img(cable_segment_mask)
    
    cable_len, cable_endpoints = find_length_and_endpoints(cable_skeleton)

    channel_start_d = (channel_start[1], channel_start[0])
    # channel_cloud, _, channel_waypoints, possible_channel_end_pts = g.segment_channel(channel_start_d)
    channel_cloud_pixels, channel_cloud, _, channel_waypoints, possible_channel_end_pts = \
        g.segment_channel(channel_start_d, use_pixel=True)
    # waypoint_first= g.ij_to_point(channel_waypoints[0]).data
    waypoint_first = channel_waypoints[0]
    print('channel cloud shape', channel_cloud.shape)
    print('channel waypoints one case:', channel_waypoints[0])
    print('channel waypoints one case adjusted', waypoint_first)
    print('channel cloud one case', channel_cloud[-1])
    print('channel cloud one case', channel_cloud.data[-1])
    print('channel cloud', channel_cloud)
    print('location test', np.where(channel_cloud.data == channel_cloud.data[-1]))
    print('channel waypoints', channel_waypoints)
    plt.scatter(x = [j[1] for j in channel_waypoints], y=[i[0] for i in channel_waypoints],c='c')
    plt.scatter(x = [j[1] for j in cable_waypoints], y=[i[0] for i in cable_waypoints],c='0.75')
    plt.scatter(x = [j[1] for j in possible_channel_end_pts], y=[i[0] for i in possible_channel_end_pts],c='0.45')
    plt.scatter(x=channel_start[1], y=channel_start[0], c='m')
    plt.scatter(x=cable_pt[1], y=cable_pt[0], c='w')
    plt.imshow(three_mat_depth, interpolation="nearest")
    plt.show()

    blank = np.zeros((len(three_mat_depth[0]), len(three_mat_depth)))
    # for every point in the ropecloud we set it to 1 so we have our cable segment
    for r,c in channel_cloud_pixels:
        blank[r][c] = 1
    image_channel_data = blank
    plt.title("image_channel_data")
    plt.imshow(image_channel_data)
    plt.show()
    image_channel_data = gaussian_filter(image_channel_data, sigma=1)
    channel_skeleton = skeletonize_img(image_channel_data)

    channel_len, channel_endpoints = find_length_and_endpoints(channel_skeleton)

# Code just picks the endpoint that's furthest from the origin as the loose end of the cable <- BAD CODE!
    # copy_channel_data = copy.deepcopy(image_channel_data)

    # img_skeleton = cv2.cvtColor(copy_channel_data, cv2.COLOR_RGB2GRAY)

    # features = cv2.goodFeaturesToTrack(img_skeleton, 10, 0.01, 200)
    # print("OPEN CV2 FOUND FEATURES: ", features)
    # endpoints = [x[0] for x in features]

    # closest_to_origin = (0, 0)
    # furthest_from_origin = (0, 0)
    # min_dist = 10000000
    # max_dist = 0
    # for endpoint in endpoints:
    #     dist = np.linalg.norm(np.array([endpoint[0], endpoint[1]-400]))
    #     if dist < min_dist:
    #         min_dist = dist
    #         closest_to_origin = endpoint
    # for endpoint in endpoints:
    #     dist = np.linalg.norm(
    #         np.array([closest_to_origin[0]-endpoint[0], closest_to_origin[1]-endpoint[1]]))
    #     if dist > max_dist:
    #         max_dist = dist
    #         furthest_from_origin = endpoint
    # endpoints = [closest_to_origin, furthest_from_origin]
    # print("ENDPOINTS SELECTED: " + str(endpoints))
    # if DISPLAY:
    #     print("img skeleton")
    #     plt.scatter(x=[j[0][0] for j in features], y = [j[0][1] for j in features], c = '0.2')
    #     plt.scatter(x=[j[0] for j in endpoints], y = [j[1] for j in endpoints], c = 'm')
    #     plt.imshow(img_skeleton, interpolation="nearest")
    #     plt.show()
    
    
    
    # looking at endpoints that differ more in depth cause that means that they're not connected 
    # (since channel is elevated)
    max_diff = 0
    min_diff = 100000000000
    disconnected_channel_endpoint = []
    disconnected_cable_endpoint = []
    connected_channel_endpoint = []
    connected_cable_endpoint = []

    for channel_endpoint in channel_endpoints:
        channel_r, channel_c = channel_endpoint[1], channel_endpoint[0]
        channel_depth = three_mat_depth[channel_r][channel_c]
        for cable_endpoint in cable_endpoints:
            cable_r, cable_c = cable_endpoint[1], cable_endpoint[0]
            cable_depth = three_mat_depth[cable_r][cable_c]
            depth_diff = abs(channel_depth - cable_depth)
            if depth_diff > max_diff:
                disconnected_cable_endpoint = cable_endpoint
                disconnected_channel_endpoint = channel_endpoint
            elif depth_diff < min_diff:
                connected_cable_endpoint = cable_endpoint
                connected_channel_endpoint = channel_endpoint
    connected_endpoints = [connected_cable_endpoint, connected_channel_endpoint]
    disconnected_endpoints = [disconnected_cable_endpoint, disconnected_channel_endpoint]
    plt.scatter(x=[j[0] for j in connected_endpoints], y=[i[1] for i in connected_endpoints], c='r')
    plt.scatter(x=[j[0] for j in disconnected_endpoints], y=[i[1] for i in disconnected_endpoints], c='r')
    plt.imshow(three_mat_depth, interpolation="nearest")
    plt.show()

                
    # once have the endpoints now need to generate evenly spaced waypoints in both cable and channel
    # question is what is the best metric for how to determine waypoints between things and have them be equidistant

    # want 5 total waypoints for entire task
    num_waypoints = 5 - num_iterations

    
    
    
    # Use estimation
    place = (0,0)

    print("ACTUAL PLACE: "+str(place))

    # increasing the number of times we've scanned the image
    num_iterations += 1


if __name__ == "__main__":
    main()