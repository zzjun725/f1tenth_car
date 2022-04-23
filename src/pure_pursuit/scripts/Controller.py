from ast import Gt
import imp
from cv2 import log
import numpy as np
import torch
import cv2
from queue import PriorityQueue
import matplotlib.pyplot as plt
import os
from time import strftime, gmtime
import csv
import fire
import ipdb
from collections import deque
from numba import njit
# from agents.utility import nearest_point_on_trajectory, get_two_point_orientation

wp_path = os.path.dirname(os.path.abspath(__file__))
print(wp_path)

class LimitedList(list):

    # Read-only
    @property
    def maxLen(self):
        return self._maxLen

    def __init__(self, *args, **kwargs):
        self._maxLen = kwargs.pop("maxLen") 
        list.__init__(self, *args, **kwargs)
        self._curLen = 0
    
    def len(self):
        return self._curLen
    
    # @njit(fastmath=False, cache=True)
    def _truncate(self):
        """Called by various methods to reinforce the maximum length."""
        dif = len(self)-self._maxLen
        if dif > 0:
            self[:dif]=[]
            return True
        return False

    def append(self, x):
        list.append(self, x)
        if not self._truncate():
            self._curLen += 1


class Road_logger:
    def __init__(self, debug=True, x_length = 512, save_log=True) -> None:
        if save_log:    
            self.log = open(strftime(wp_path+'/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        self.old_mid = x_length // 2

    def resize(self, img, w=512, h=256):
        resize_img = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
        return resize_img
    
    def save_wp(self, x, y, v, targetV):
        self.log.write('%f, %f, %f, %f\n' % (x, y, v, targetV))
    
    def load_wp(self, wp_path=wp_path):
        for file in os.listdir(wp_path):
            if file.startswith('wp'):
                wp_log = csv.reader(open(os.path.join(wp_path, file)))
                print('load wp_log')
                break

        points = [] # (x, y, theta)
        for i, row in enumerate(wp_log):
            points.append([float(row[0]), float(row[1])])
        return points
    
    def load_Wp(self, wp_file=None):
        # for file in os.listdir(wp_path):
        #     if file.startswith('wp'):
        wp_log = csv.reader(open(os.path.join(wp_path, wp_file)))
                # print('load wp_log')
                # break

        points = [] # (x, y, theta)
        for i, row in enumerate(wp_log):
            points.append([float(row[1]), float(row[0]), float(row[2]), float(row[3])])
        return np.array(points)
    
    def load_OptimalWp(self, wp_path=wp_path): 
        self.wp=[]
        for file in os.listdir(os.path.join(wp_path, 'wp')):
            if file.startswith('Optimalwp'):
                wp_log = csv.reader(open(os.path.join(wp_path, 'wp', file)))
                print('load Optimalwp_log')
                break
        for i, row in enumerate(wp_log):
            if i > 2:
                # import ipdb; ipdb.set_trace()
                row = row[0].split(';')
                x, y, v = float(row[1]), float(row[2]), np.clip(float(row[5])/2.5, 0, 15)
                # self.wp.append([x, y, v])
                # TODO: fix this if the road logger is correct
                self.wp.append([y, x, v])
        self.wp = np.array(self.wp).T  # (3, n), n is the number of waypoints
        return self.wp.T    
    
    def draw_wp(self, points):
        fig, ax = plt.subplots()
        x = []
        y = []
        for point in points:
            x.append(point[0])
            y.append(point[1])
        ax.plot(x, y, '-ro')
        plt.show()
        plt.pause(100)
    
    def gen_drift(self, right_drift=3.5, left_drift=3.5, file_name=1):
        for file in os.listdir(wp_path):
            if file.startswith('wp'):
                wp_log = csv.reader(open(os.path.join(wp_path, file)))
                print('load wp_log')
                break

        drift_log = open(os.path.join(wp_path, f'wp_drift{file_name}.csv'), 'w')
        points = [] # (x, y, theta)
        for i, row in enumerate(wp_log):
            points.append([float(row[0]), float(row[1]), right_drift, left_drift])
        for i, row in enumerate(points):
            drift_log.write('%f, %f, %f, %f\n' % (row[0], row[1], row[2], row[3]))


class TrackingPlanner:
    def __init__(self, wp_path=wp_path, wp_gap = 3, 
                 drawW=400, drawH=800, drawLen=50, debug=True):
        # wp
        self.wp = []
        self.wpNum = None
        self.wp_path = wp_path
        self.wpGapCounter = 0
        self.wpGapThres = wp_gap
        self.max_speed = 70
        self.speedScale = 1.0

        # draw
        self.wp_buffer = LimitedList(maxLen=drawLen)
        self.position_buffer = LimitedList(maxLen=drawLen)
        self.scale = 10
        self.drawW = drawW
        self.drawH = drawH
        self.debug = debug

        # PID for speed
        self.Increase_P = 1 / 5
        self.Decrease_P = 1 / 6
        self.P = 10
        self.targetSpeed = 0

    def load_wp(self):
        for file in os.listdir(wp_path):
            if file.startswith('wp'):
                wp_log = csv.reader(open(os.path.join(wp_path, file)))
                print(f'load wp_log: {file}')
                break

        points = [] # (x, y, theta)
        # TODO: fix this
        wp_nums = len(wp_log) 
        for i, row in enumerate(wp_log):
            points.append([float(row[1]), float(row[0]), self.max_speed-5])
        self.wp = np.array(points).T
    
    def load_Optimalwp(self): 
        for file in os.listdir(os.path.join(wp_path, 'wp')):
            if file.startswith('optimal'):
                wp_log = csv.reader(open(os.path.join(wp_path, 'wp', file)))
                print('load Optimalwp_log')
                break
        for i, row in enumerate(wp_log):
            if i > 2:
                # import ipdb; ipdb.set_trace()
                if self.wpGapCounter == self.wpGapThres:
                    self.wpGapCounter = 0
                    row = row[0].split(';')
                    x, y, v = float(row[1]), float(row[2]), np.clip(float(row[5])/self.speedScale, 0, self.max_speed)
                    # self.wp.append([x, y, v])
                    # TODO: fix this if the road logger is correct
                    self.wp.append([y, x, v])
                else:
                    self.wpGapCounter += 1
        self.wp = np.array(self.wp).T  # (3, n), n is the number of waypoints
        self.wpNum = len(self.wp[0])
    
    def pidAccel(self, diff, targetS=0, curS=0):
        a = self.P * diff
        if a > 0 :
            a = self.Increase_P * a
        else: 
            a = self.Decrease_P * a
        print(f'a: {np.round(a, 3)}')
        a = np.clip(a, -1.0, 1.0)
        print(f'a: {np.round(a, 3)}')
        return np.clip(a, -1.0, 1.0)
    
    
    def planning(self, pose, speed):
        """
        pose: (global_x, global_y, yaw) of the car
        speed: current speed of the car

        Return:
        steering_angle, accelation
        """
        raise NotImplementedError
        # return steering_angle, accelation
    
    # Yellow:  [0, 255, 255]
    # cv2.polylines(img, pts, isClosed, color[, thickness[, lineType[, shift]]])
    # @njit(fastmath=False, cache=True)
    def cal_traj(self, yaw=None):
        # wp_buffer = np.floor(np.array(self.wp_buffer)*self.scale).T  # (2, n)
        # position_buffer = np.floor(np.array(self.position_buffer)*self.scale).T  # (2, n)
        
        wp_buffer = np.floor(np.array(self.wp_buffer)*self.scale).astype(np.int64)  # (n, 2)
        position_buffer = np.floor(np.array(self.position_buffer)*self.scale).astype(np.int64)  # (n, 2)        
        # R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        # inv
        # R = np.linalg.inv(R)
        # wp_buffer = (R @ wp_buffer).T.astype(np.int64)  # (n, 2)
        # position_buffer = (R @ position_buffer).T.astype(np.int64)  # (n, 2)

        W, H = self.drawW, self.drawH
        
        wp_buffer[:, 1] = - wp_buffer[:, 1]
        position_buffer[:, 1] = -position_buffer[:, 1]

        xy_min = np.min(np.vstack([wp_buffer, position_buffer]), axis=0).reshape(-1, 2)
        wp_buffer = wp_buffer - xy_min
        position_buffer = position_buffer - xy_min

        xy_max = np.max(np.vstack([wp_buffer, position_buffer]), axis=0).reshape(-1, 2)
        drawOrigin = np.array([W//2, H]).reshape(1, 2) - xy_max 
        wp_buffer = wp_buffer + drawOrigin
        position_buffer = position_buffer + drawOrigin
        return wp_buffer, position_buffer
    
    def show_traj(self, wait=5, yaw=None):
        wp_buffer, position_buffer = self.cal_traj(yaw)

        W, H = self.drawW, self.drawH
        blank_image = np.ones((H, W, 3), np.uint8)
        blank_image.fill(255)
        cv2.polylines(blank_image, [wp_buffer], False, (0, 0, 255))
        cv2.polylines(blank_image, [position_buffer], False, (255, 0, 0))
        cv2.imshow('debug', blank_image)
        cv2.waitKey(wait)
    
    def update_traj(self, targetWp, pose):
        if self.debug:
            self.wp_buffer.append(targetWp)
            self.position_buffer.append(pose[:2])
            if len(self.wp_buffer) == 50:
                self.show_traj(yaw=pose[2])    


class PurePursuitPlanner(TrackingPlanner):
    def __init__(self, debug=True):
        super().__init__(wp_path=wp_path, wp_gap=0, drawW=400, drawH=800, debug=debug)
        # self.wp = []
        self.minL = 1.5
        self.maxL = 3.7
        self.minP = 0.7
        self.maxP = 1.0
        self.interpScale = 20
        self.Pscale = 30
        self.Lscale = 25
        self.interp_P_scale = (self.maxP-self.minP) / self.Pscale
        self.interp_L_scale = (self.maxL-self.minL) / self.Lscale
        self.prev_error = 0
        self.D = 0.15 
        self.errthres = 0.1
        self.load_Optimalwp()
    
    def find_targetWp(self, cur_position, targetL):
        """
        cur_positon: (2, )
        return: cur_L, targetWp(2, ), targetV 
        """
        # ipdb.set_trace()

        wp_xyaxis = self.wp[:2]  # (2, n)
        dist = np.linalg.norm(wp_xyaxis-cur_position.reshape(2, 1), axis=0)
        nearst_idx = np.argmin(dist)
        nearst_point = wp_xyaxis[:, nearst_idx]

        segment_end = nearst_idx
        for i, point in enumerate(wp_xyaxis.T[nearst_idx:]):
            cur_dist = np.linalg.norm(cur_position-point)
            if cur_dist > targetL:
                break
        segment_end += i
        interp_point = np.array([wp_xyaxis[0][segment_end], wp_xyaxis[1][segment_end]])
        # get interpolation
        # error = 0.1
        x_array = np.linspace(wp_xyaxis[0][segment_end-1], wp_xyaxis[0][segment_end], self.interpScale)
        y_array = np.linspace(wp_xyaxis[1][segment_end-1], wp_xyaxis[1][segment_end], self.interpScale)
        v_array = np.linspace(self.wp[2][segment_end-1], self.wp[2][segment_end], self.interpScale)
        xy_interp = np.vstack([x_array, y_array])
        dist_interp = np.linalg.norm(xy_interp-cur_position.reshape(2, 1), axis=0) - targetL
        i_interp = np.argmin(np.abs(dist_interp))
        interp_point = np.array([x_array[i_interp], y_array[i_interp]])
        interp_v = v_array[i_interp]
        cur_L = np.linalg.norm(cur_position-interp_point)
        # ipdb.set_trace()
        return cur_L, interp_point, interp_v, segment_end, nearst_point
    
    def planning(self, pose, speed):
        """
        pose: (global_x, global_y, yaw) of the car
        """
        targetL = speed * self.interp_L_scale + self.minL
        P = self.maxP - speed * self.interp_P_scale 
        
        yaw = - pose[2]
        pose[2] = yaw
        local2global = np.array([[np.cos(yaw), -np.sin(yaw), 0, pose[0]], 
                                 [np.sin(yaw), np.cos(yaw), 0, pose[1]], 
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])        
        
        # wp_xyaxis = self.wp[:2]
        cur_L, targetWp, targetV, segment_end, nearstP = self.find_targetWp(pose[:2], targetL)

        global2local = np.linalg.inv(local2global)
        nearstP_local = global2local @ np.array([nearstP[0], nearstP[1], 0, 1]) 
        cur_error = nearstP_local[0]

        offset = - self.D * (cur_error - self.prev_error)
        self.prev_error = cur_error

        local_goalP = global2local @ np.array([targetWp[0], targetWp[1], 0, 1])
        gamma = 2*abs(local_goalP[0]) / (cur_L ** 2)
        if local_goalP[0] < 0:
            steering_angle = P * gamma
        else:
            steering_angle = P * -gamma
        steering_angle = np.clip(steering_angle+offset, -1.0, 1.0)
        self.targetSpeed = targetV
        diff = targetV - speed
        acceleration = self.pidAccel(diff)

        return steering_angle, acceleration    


class StanleyPlanner(TrackingPlanner):
    def __init__(self, wp_path=wp_path, wp_gap=3, 
                 drawW=400, drawH=800, drawLen=10, debug=True,
                 wb=0.3302, kv=8):
        super().__init__(wp_path, wp_gap, drawW, drawH, drawLen, debug)
        self.wheelbase = wb
        self.kv = kv
        self.load_Optimalwp()
        self.P = 0.4
        self.headSpan = 2

    def _get_average_k(self, p):
        p1 = p[:, :-1]  # (2, n-1)
        p2 = p[:, 1:]  # (2, n-1)
        delta = p2 - p1 # (2, n-1)
        k = np.arctan2(delta[1], delta[0])
        k = (k + 2*np.pi) % (2*np.pi)
        k = np.mean(k)
        return k
    
    def _get_current_waypoint_and_heading(self, position):
        wpts = self.wp[:2]
        # nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts.T)
        wp_xyaxis = self.wp[:2]  # (2, n)
        dist = np.linalg.norm(wp_xyaxis-position.reshape(2, 1), axis=0)
        i = np.argmin(dist)
        nearest_point = wp_xyaxis[:, i]  # (2, )
       
        begin = np.clip(i-self.headSpan, 0, self.wpNum-2)
        end = np.clip(i+self.headSpan, 1, self.wpNum-1)
        heading = self._get_average_k(wp_xyaxis[:, begin:end+1])

        return nearest_point, heading, i 

    @staticmethod
    def _calculate_angle_difference(start, end):
        a = end - start
        # a = (a + np.pi) % (2*np.pi) - np.pi
        return a

    def planning(self, pose, speed):
        position = np.array([pose[0], pose[1]])  # (2, )
        cur_heading = (-pose[2] + np.pi/2) % (2*np.pi)  # (y is the heading)
        nearest_point, target_heading, i = self._get_current_waypoint_and_heading(position)
        # target_heading = (target_heading + 2*np.pi) % (2*np.pi)
        
        # speed 
        target_speed = self.wp[2][i]
        accelation = self._acceleration_logic(target_speed, speed, self.max_speed)

        # the cur_heading and target_heading all in (0, 2*np.pi)
        heading_error = self._calculate_angle_difference(cur_heading, target_heading)
        
        # distance error
        d_nearest = nearest_point - position
        y_Unitvector = np.array([np.cos(cur_heading), np.sin(cur_heading)])
        nearest_dist = np.dot(d_nearest, y_Unitvector)

        # nearest_dist = np.dot(d_nearest, [np.cos(pose[2] + np.pi / 2), np.sin(pose[2] + np.pi / 2)])
        # ipdb.set_trace()
        distance_error = np.arctan2(self.kv*nearest_dist, speed+1)
        steering_angle = heading_error+distance_error/2
        steering_angle = steering_angle * self.P

        # ipdb.set_trace()
        print(f'steering_angle{steering_angle}')
        print(f'heading_error{heading_error}')
        print(f'distance_error{distance_error}')
        print(f'nearest_dist{nearest_dist}')
        self.update_traj(targetWp=self.wp[:2, i], pose=pose)

        return np.clip(steering_angle, -1.0, 1.0), accelation   


def draw_wp():
    filter = Road_logger(save_log=False)
    points = filter.load_wp()
    fig, ax = plt.subplots()
    x = []
    y = []
    for point in points:
        x.append(point[0])
        y.append(point[1])
    # TODO: fix if x, y is right
    ax.plot(y, x, '-ro', )
    plt.show()

def generate_drift():
    filter = Road_logger(save_log=False)
    filter.gen_drift()

def draw_raceline():
    outputdir = '/home/mlab/zhijunz/global_racetrajectory_optimization/outputs'
    for file in os.listdir(outputdir):
        raceline_log = csv.reader(open(os.path.join(outputdir, file)))
    # import ipdb; ipdb.set_trace()
    x = []
    y = []
    for i, row in enumerate(raceline_log):
        if i > 2:
            # import ipdb; ipdb.set_trace()
            row = row[0].split(';')
            x.append(float(row[1]))
            y.append(float(row[2]))
    fig, ax = plt.subplots()
    ax.plot(x, y, '-b', )
    plt.show()

def analyze_error(racelinePath=wp_path, trajPath='wp_25s.csv'):
    wpLogger = Road_logger(save_log=False)
    gt = wpLogger.load_OptimalWp(racelinePath).T  # (3, n)
    curr = wpLogger.load_Wp(trajPath).T  # (4, n)
    gt_velocity = curr[-1]

    gtPoints = gt[:2]
    curPoints = curr[:2]

    # error
    errors = []
    for point in curPoints.T:
        point = point.reshape(2, 1)
        dist = np.linalg.norm(gtPoints-point, axis=0) # (n, )
        error = np.min(dist)
        errors.append(error)
    
    # velocity
    velocity = curr[2]
    
    fig = plt.subplot(111)
    plt.plot(gt[0], gt[1], '-r', lineWidth=0.5, label='raceline')
    plt.plot(curr[0], curr[1], '-b', lineWidth=0.5, label='current traj')
    plt.show()
    
    fig = plt.subplot(111)
    plt.plot(gt_velocity, '-g', label='groundtruth_velocity')
    plt.plot(velocity, '-b', label='velocity')
    plt.plot(errors, '-r', label='errors')
    plt.legend()
    plt.show()

            
if (__name__) == '__main__':
    fire.Fire(
        {'draw': draw_wp,
        'gen_drift': generate_drift,
        'draw_raceline': draw_raceline,
        'analyze': analyze_error}
    )

    # plt.pause(100)
