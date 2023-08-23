from autolab_core import RigidTransform, PointCloud, RgbdImage, ColorImage, DepthImage
from autolab_core.image import BINARY_IM_DEFAULT_THRESH
from numpy.lib.histograms import histogram_bin_edges
from collision import CollisionInterface
from queue import Empty
from yumiplanning.yumi_kinematics import YuMiKinematics
from yumiplanning.yumi_planner import Planner
import numpy as np
from multiprocessing import Queue, Process
from random import shuffle
import math

class GraspSegmenter:

    #  zed camera gives us rgb and depth separately so we just save them differently
    def __init__(self, rgb_image, depth_image):
        self.color = rgb_image
        self.depth = depth_image

    def segment_cable(self, loc, orient_mode=False):
        '''
        returns a PointCloud corresponding to cable points along the provided location
        inside the depth image
        '''
        q = [loc]
        pts = []
        closepts = []
        visited = set()
        print("Loc", loc)
        #  DOUBLE CHECK WHETHER OR NOT IT IS (loc[1], loc[0]) or NOT!!!!
        start_point = self.depth[loc[1]][loc[0]]
        print("start_point", start_point)

        start_color = self.color[loc[1]][loc[0]]
        RADIUS2 = 1  # distance from original point before termination
        CLOSE2 = .002**2
        DELTA = .00080#.00075
        NEIGHS = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        COLOR_THRESHOLD = 45
        counter = 0
        waypoints = []
        weird_pts = []
        # carry out floodfill
        while len(q) > 0:
            # if we're in orient_mode we only want the local segment of the cable to find the principle axis, not the entire cable
            if (orient_mode):
                if counter > 600:
                    break
            next_loc = q.pop()
            next_point = self.depth[next_loc[1]][next_loc[0]]
            # NOT SURE IF THIS IS NECESARY
            next_color = np.asarray(self.color[next_loc[1]][next_loc[0]])
            next_color = next_color.astype(np.int16)
            visited.add(next_loc)
            diff = start_point-next_point
            dist = diff.dot(diff)
            if (dist > RADIUS2):
                continue
            pts.append(next_point)
            # has us updating the list of points to our waypoint
            if counter % 1000 == 0:
                waypoints.append((next_loc[1],next_loc[0]))
            counter += 1
            if (dist < CLOSE2):
                closepts.append(next_point)
            # add neighbors if they're within delta of current height
            for n in NEIGHS:
                test_loc = (next_loc[0]+n[0], next_loc[1]+n[1])
                if test_loc[0] >= self.depth.width or test_loc[0] < 0 \
                        or test_loc[1] >= self.depth.height or test_loc[1] < 0:
                    continue
                if (test_loc in visited):
                    continue
                # want to check if the points were adding are of similar color cause the cable is a uniform color
                # the channel currently is not the same color so this is another method to differentiate between them
                test_pt = self.ij_to_point(test_loc).data
                # this is cause the data is grayscaled so we can just look at the Red channel 
                test_color = np.asarray(self.color[test_loc[1]][test_loc[0]])
                # pdb.set_trace()
                test_color = test_color.astype(np.int16)
                color_diff = np.linalg.norm(next_color-test_color)
                if (20 < color_diff < 45):
                    weird_pts.append([test_loc[1], test_loc[0]])
                if (abs(test_pt[2]-next_point[2]) < DELTA and color_diff < COLOR_THRESHOLD):
                    q.append(test_loc)

        return np.array(pts), np.array(closepts), waypoints, weird_pts

    def segment_channel(self, loc, use_pixel = False):
        '''
        returns a PointCloud corresponding to cable points along the provided location
        inside the depth image
        '''
        q = [loc]
        pts = []
        pts_pixels = []
        closepts = []
        visited = set()
        start_point = self.depth[loc[1]][loc[0]]
        RADIUS2 = 1  # distance from original point before termination
        CLOSE2 = .002**2
        DELTA = 0.0002*3
        NEIGHS = [(-1, 0), (1, 0), (0, 1), (0, -1)]
        counter = 0
        waypoints = []
        endpoints = []
        # carry out floodfill
        while len(q) > 0:
            next_loc = q.pop()
            next_point = self.depth[next_loc[1]][next_loc[0]]
            # Next Point Example: [0.33499247 0.00217638 0.05645943]
            visited.add(next_loc)
            diff = start_point-next_point
            dist = diff.dot(diff)
            if (dist > RADIUS2):
                continue
            pts_pixels.append(next_loc)
            pts.append(next_point)
            # has us updating the list of points to our waypoint
            if counter % 800 == 0:
                waypoints.append((next_loc[1],next_loc[0]))
            counter += 1
            if (dist < CLOSE2):
                closepts.append(next_point)
            # add neighbors if they're within delta of current height
            for n in NEIGHS:
                test_loc = (next_loc[0]+n[0], next_loc[1]+n[1])
                if (test_loc in visited):
                    continue
                if test_loc[0] >= self.depth.width or test_loc[0] < 0 \
                        or test_loc[1] >= self.depth.height or test_loc[1] < 0:
                    continue
                test_pt = self.ij_to_point(test_loc).data
                if (abs(test_pt[2]-next_point[2]) < DELTA):
                    q.append(test_loc)
        if (use_pixel == False):
            # NOT SURE WHETHER OT TRANSPOSE OR NOT
            return np.array(pts), np.array(closepts), waypoints, endpoints
        else:
            return pts_pixels, np.array(pts), np.array(closepts), waypoints, endpoints
        