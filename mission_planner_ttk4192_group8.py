#!/usr/bin/env python3
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan
from os import system, name
# from time import time
import time
import re
import fileinput
import sys
import argparse
import random
import matplotlib.animation as animation
from datetime import datetime
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy

# Import here the packages used in your codes
# from utils.grid import Grid_robplan
from utils.car import SimpleCar
from utils.environment import Environment_robplan
from utils.grid import Grid_robplan
from utils.dubins_path import DubinsPath

# Moving the arm
from utils.arm_control import move_arm, control_gripper
""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 


# 1) Program here your AI planner (only one) ------------------------------------------------------------
"""
1) Temporal planner : Program a routine wich call the installed STP planner; 
2) Graph plan       : Use the graph-plan code provided in the lecture
3) other algorithm 
"""

# STP Temporal planner
def read_plan():
    plan_general = []
    with open(os.path.expanduser('~/catkin_ws/src/AI-planning/tmp_sas_plan.1')) as fp:
        for entry in fp:
            print(entry)
            task = entry.split("(")[1].split(")")[0][1:-1]
            print(task)
            plan_general.append(task)
    print(plan_general[0])
    return plan_general


# 2) Program here your path-finding algorithm (only one) --------------------------------------------------------------------
""" 
1) Hybrid A-star pathfinding : Use the algorithm provided in Assignment 1
2) A-star                    : Program your code
3) Other method
"""



class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

# ----------------- Copied from CA1 ------------------------
class HybridAstar:
    """ Hybrid A* search procedure. """
    def __init__(self, car, grid_robplan, reverse, unit_theta=pi/12, dt=1e-2, check_dubins=1):
        
        self.car = car
        self.grid = grid_robplan
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.drive_steps = int(sqrt(2)*self.grid.cell_size/self.dt) + 1
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        self.dubins = DubinsPath(self.car)
        self.astar = Astar(self.grid, self.goal[:2])
        
        self.w1 = 0.95 # weight for astar heuristic
        self.w2 = 0.05 # weight for simple heuristic
        self.w3 = 0.30 # weight for extra cost of steering angle change
        self.w4 = 0.10 # weight for extra cost of turning
        self.w5 = 2.00 # weight for extra cost of reversing

        self.thetas = get_discretized_thetas(self.unit_theta)
    
    def construct_node(self, pos):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        theta = round_theta(theta % (2*pi), self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        grid_pos = cell_id + [theta]

        node = Node(grid_pos, pos)

        return node
    
    def simple_heuristic(self, pos):
        """ Heuristic by Manhattan distance. """

        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])
        
    def astar_heuristic(self, pos):
        """ Heuristic by standard astar. """

        h1 = self.astar.search_path(pos[:2]) * self.grid.cell_size
        h2 = self.simple_heuristic(pos[:2])
        
        return self.w1*h1 + self.w2*h2

    def get_children(self, node, heu, extra):
        """ Get successors from a state. """

        children = []
        for m, phi in self.comb:

            # don't go back
            if node.m and node.phi == phi and node.m*m == -1:
                continue

            if node.m and node.m == 1 and m == -1:
                continue

            pos = node.pos
            branch = [m, pos[:2]]

            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m)
                branch.append(pos[:2])

            # check safety of route-----------------------
            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)
            # --------------------------------------------
            
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                # extra cost for changing steering angle
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                
                # extra cost for turning
                if phi != 0:
                    child.g += self.w4 * self.arc
                
                # extra cost for reverse
                if m == -1:
                    child.g += self.w5 * self.arc

            if heu == 0:
                child.f = child.g + self.simple_heuristic(child.pos)
            if heu == 1:
                child.f = child.g + self.astar_heuristic(child.pos)
            
            children.append([child, branch])

        return children
    
    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        """ Search best final shot in open set. """

        open_.sort(key=lambda x: x.f, reverse=False)

        for t in range(min(n, len(open_))):
            best_ = open_[t]
            solutions_ = self.dubins.find_tangents(best_.pos, self.goal)
            d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
        
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best = best_
                cost = cost_
                d_route = d_route_
        
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        
        return best, cost, d_route
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start)
        root.g = float(0)
        root.g_ = float(0)
        
        if heu == 0:
            root.f = root.g + self.simple_heuristic(root.pos)
        if heu == 1:
            root.f = root.g + self.astar_heuristic(root.pos)

        closed_ = []
        open_ = [root]

        count = 0
        while open_:
            count += 1
            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check dubins path
            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(best.pos, self.goal)
                d_route, cost, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    best, cost, d_route = self.best_final_shot(open_, closed_, best, cost, d_route)
                    route = self.backtracking(best) + d_route
                    path = self.car.get_path(self.start, route)
                    cost += best.g_
                    print('Shortest path: {}'.format(round(cost, 2)))
                    print('Total iteration:', count)
                    
                    return path, closed_

            children = self.get_children(best, heu, extra)

            for child, branch in children:

                if child in closed_:
                    continue

                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)

                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)

                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    
                    open_.remove(child)
                    open_.append(child)

        return None, None
# ----------------- Copied from CA1 ------------------------


def main_hybrid_a(heu,start_pos, end_pos,reverse, extra, grid_on):
    print(f"REVERSE: {reverse}")
    tc = map_grid_robplan()
    env = Environment_robplan(tc.obs, lx=5.0*10, ly=2.9*10)
    # env = Environment_robplan(lx=20)
    car = SimpleCar(env, start_pos, end_pos, l=0.5)
    grid = Grid_robplan(env)

    hastar = HybridAstar(car, grid, reverse)
    # hastar = HybridAstar(car, grid, reverse, dt=0.1)

    t = time.time()
    path, closed_ = hastar.search_path(heu, extra)
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    if not path:
        print('No valid path!')
        return
    # a post-processing is required to have path list
    path = path[::5] + [path[-1]]
    #for i in range(len(path)):
    #    print(path[i].pos[0])
    
    branches = []
    bcolors = []
    for node in closed_:
        for b in node.branches:
            branches.append(b[1:])
            bcolors.append('y' if b[0] == 1 else 'b')

    xl, yl = [], []
    xl_np1,yl_np1=[],[]
    carl = []
    dt_s=int(25)  # samples for gazebo simulator
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])
        if i==0 or i==len(path):
            xl_np1.append(path[i].pos[0])
            yl_np1.append(path[i].pos[1])            
        elif dt_s*i<len(path):
            xl_np1.append(path[i*dt_s].pos[0])
            yl_np1.append(path[i*dt_s].pos[1])      
    # defining way-points (traslandado el origen a (0,0))
    # xl_np=np.array(xl_np1)
    # xl_np=xl_np-20
    # yl_np=np.array(yl_np1)
    # yl_np=yl_np-11.2

    xl_np=np.array(xl_np1)
    xl_np=xl_np/10
    yl_np=np.array(yl_np1)
    yl_np=yl_np/10
    global WAYPOINTS_MOVE
    WAYPOINTS_MOVE=np.column_stack([xl_np,yl_np])
    #print(WAYPOINTS)
    
    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid(which='both')
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    print("Printing waypoints")
    print()
    for wp in WAYPOINTS:
        print(wp)
        ax.plot(wp[0], wp[1], 'ro', markersize=6)
    print()

    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    # _branches = LineCollection(branches, color='b', alpha=0.8, linewidth=1)
    # ax.add_collection(_branches)

    # _carl = PatchCollection(carl[::20], color='m', alpha=0.1, zorder=3)
    # ax.add_collection(_carl)
    # ax.plot(xl, yl, color='whitesmoke', linewidth=2, zorder=3)
    # _car = PatchCollection(path[-1].model, match_original=True, zorder=4)
    # ax.add_collection(_car)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _path, _carl, _path1, _car

    def animate(i):

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _branches.set_color(bcolors)
        
        else:
            _branches.set_paths(branches)

            j = i - len(branches)

            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_edgecolor('k')
            _carl.set_facecolor('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)

            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)

            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)

        return _branches, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                  interval=10, repeat=True, blit=True)

    plt.show()

# Create a map grid here for Hybrid A* 

class map_grid_robplan:
    def __init__(self):
        # x and y are the coordinates for the lower left corener
        # w, h are width and height
        # [x, y, w, h]
        scale = 10
        # MIDDLE BOXES
        obs = [
            [1.0, 1.5, 0.4, 0.8],
            [1.5, 1.5, 0.4, 0.8],
            [2.0, 1.5, 0.7, 0.8]
        ]

        # Obstacles defined by waypoints
        obs.extend([
            [WAYPOINTS[6][0]/scale-0.4, WAYPOINTS[6][1]/scale+0.2, 0.8, 0.5],
            [WAYPOINTS[1][0]/scale-0.45, (WAYPOINTS[1][1]+2)/scale, 0.9, 0.3],
            [WAYPOINTS[1][0]/scale-0.45+0.9+0.2, (WAYPOINTS[1][1]+1)/scale, 0.9, 0.3],
        ])

        # Addind the wall obstacles
        obs.extend([
            [0, 1.0, 0.5, 0.2],
            [3.0, 0.0, 2, 0.3]
        ])
        obs_ = scale * np.array(obs)
        self.obs = obs_.tolist()
        
#3) GNC module (path-followig and PID controller for the robot) ------------------------------
"""  Robot Guidance navigation and control module 
"""
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        PI = 3.1415926535897
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    """
    Path-following module
    """
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        # track a sequence of waypoints
        for point in WAYPOINTS_MOVE:
            # Ugly but necessary
            # self.move_to_point(point[0] * 0.1, point[1] * 0.1)
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()


    def move_to_point(self, x, y):
        # Here must be improved the path-following ---
        # Compute orientation for angular vel and direction vector for linear velocity

        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)

        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        
        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = 1.5*linear   # Here can adjust speed
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()


    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)
        

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            #rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))




#4) Program here the turtlebot actions (based in your PDDL domain)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""
1
class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False
        
def taking_photo_exe():
    # Initialize
    camera = TakePhoto()

    # Default value is 'photo.jpg'
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    img_title = rospy.get_param('~image_title', 'photo'+dt_string+'.jpg')

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")
	#eog photo.jpg
    # Sleep to give the last log messages time to be sent

	# saving photo in a desired directory
    file_source = '~/catkin_ws/'
    file_destination = '~/catkin_ws/src/assigment4_ttk4192/scripts'
    g='photo'+dt_string+'.jpg'

    shutil.move(file_source + g, file_destination)
    rospy.sleep(1)

def move_robot_waypoint0_waypoint1():
    # This function executes Move Robot from 1 to 2
    # This function uses hybrid A-star
    # a=0
    # while a<3:
    #     print("Excuting Mr12")
    #     time.sleep(1)
    #     a=a+1
    print("Computing hybrid A* path")
	
    p = argparse.ArgumentParser()
    p.add_argument('-heu', type=int, default=1, help='heuristic type')
    p.add_argument('-r', action='store_true', help='allow reverse or not')
    p.add_argument('-e', action='store_true', help='add extra cost or not')
    p.add_argument('-g', action='store_true', help='show grid or not')
    args = p.parse_args()
    # start_pos = [10*0.2, 10*0.2, 0]
    # end_pos = [2.3, 1.3, 0]
    start_pos = [1, 1, 0]
    end_pos = [1.1, 1, 0]
    main_hybrid_a(args.heu,start_pos,end_pos,args.r,args.e,args.g)
    print("Executing path following")
    turtlebot_move()



def Manipulate_OpenManipulator_x():
    print("Executing manipulate a weight")
# -------------- Example from arm_control.py ------------------------
    # Move the arm to the initial position
    move_arm([0.0, 0.0, 0.0, 0.0], duration=2.0)
    rospy.sleep(2)

    # Open the gripper
    control_gripper(0.01, duration=1.0)  # Adjust the position value as needed
    rospy.sleep(1)

    # Close the gripper
    control_gripper(-0.01, duration=1.0)  # Adjust the position value as needed
    rospy.sleep(1)

    # Move the arm to home position
    move_arm([0.0, -1.0, 0.3, 0.7], duration=2.0)  # Replace with desired joint angles
    rospy.sleep(2)
    time.sleep(5)
# -------------- Example from arm_control.py ------------------------


def making_turn_exe():
    print("Executing Make a turn")
    time.sleep(1)
    #Starts a new node
    #rospy.init_node('turtlebot_move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    speed = 5
    angle = 180
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0   #should be from the odometer

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()

def check_pump_picture_ir_waypoint0():
    a=0
    while a<3:
        print("Taking IR picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

def check_seals_valve_picture_eo_waypoint0():
    a=0
    while a<3:
        print("Taking EO picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

# Charging battery 
def charge_battery_waypoint0():
    print("charging battery")
    time.sleep(5)

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

def move_robot_experiment(target):
    navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    navclient.wait_for_server()
    # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
    zaxis = [0, 0, 1]
    angle = target[2]
    qz = tf.quaternion_about_axis(angle, zaxis)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.position.z = 0.0
    
    goal.target_pose.pose.orientation.x = qz[0]
    goal.target_pose.pose.orientation.y = qz[1]
    goal.target_pose.pose.orientation.z = qz[2]
    goal.target_pose.pose.orientation.w = qz[3]

    navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = navclient.wait_for_result()
    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo ( navclient.get_result())


def move_robot(start_pos, end_pos, experiment=False):
    if experiment:
        move_robot_experiment(end_pos)
    else:
        print("Computing hybrid A* path")
        print(f"From: {start_pos} to {end_pos}")
        main_hybrid_a(1, start_pos=start_pos, end_pos=end_pos, 
                      reverse=True, 
                      extra=True,
                      grid_on=False)
        turtlebot_move()

def task6b():
    # Waypoint 1 to 2, 2 to 3, 3 to 4
    points = [[18.5, 5.0, 0], [30, 9, 0], [32.5, 26.5, pi/2]]
    
    for i, point in enumerate(points):
        if i == (len(points)-1):
            return
        start_pos = point
        end_pos = points[i+1]
        move_robot(start_pos=start_pos, end_pos=end_pos)

# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
global WAYPOINTS_MOVE
global WAYPOINTS_DICT
global WAYPOINTS
WAYPOINTS_DICT = {}
# WAYPOINTS = [[1,1],[2,2]]
# ORIENTATIONS = 
WAYPOINTS = [[0.2, 0.2, 0], [1.85, 0.6-0.1, pi/2], [3, 0.91+0.1, 3*pi/2], 
             [3.25, 2.6, pi/2], [4.8, 0.4, 0], [0.87, 2.4, pi], [3.65, 1.7-0.1, pi/2], 
             [8, 5, pi/4], [23.8, 4.3, pi/2], [24.2, 12.2, 0], 
             [30.2, 13.5, pi/2], [39, 10, pi]]
print(WAYPOINTS)
WAYPOINTS = np.array(WAYPOINTS)
WAYPOINTS[:7,:2] = WAYPOINTS[:7,:2] * 10
WAYPOINTS = WAYPOINTS.tolist()

switch_Dict = {}

def map_direction(command : str):
    # r25
    print(f"map_direction {command}")

    print(f"Befire: {WAYPOINTS[5]}")
    # 7->5 and 5->7
    if command == 'r21':
        WAYPOINTS[5][2] = pi/2
        print(f"After: {WAYPOINTS[5]}")
    if command == 'r20':
        WAYPOINTS[7][2] = 0
    
    # 2->9 and 9->2
    if command == 'r06':
        WAYPOINTS[7][2] = pi
    if command == 'r09':
        WAYPOINTS[2][2] = pi/4
    

    # 3->10 and 10->3
    if command == 'r13':
        WAYPOINTS[10][2] = 7*pi/4
    if command == 'r15':
        WAYPOINTS[3][2] = pi/2
    

    # 10->2 and 2->10
    if command == 'r10':
        WAYPOINTS[2][2] = 3*pi/2
    if command == 'r07':
        WAYPOINTS[10][2] = pi/4
    

    # 10->6 and 6->10
    if command == 'r24':
        WAYPOINTS[6][2] = 0
    if command == 'r22':
        WAYPOINTS[10][2] = pi

    # 6->11 and 11->6
    if command == 'r23':
        WAYPOINTS[11][2] = 3*pi/2
    if command == 'r25':
        WAYPOINTS[6][2] = 3*pi/4
    
    # 4->11 and 11->4
    if command == 'r17':
        WAYPOINTS[11][2] = 3*pi/4
    if command == 'r19':
        WAYPOINTS[4][2] = 7*pi/4
    

    # 11->2 and 2->11
    if command == 'r11':
        WAYPOINTS[2][2] = pi
    if command == 'r08':
        WAYPOINTS[11][2] = 0

    
    # 9->8 and 8->9
    if command == 'r27':
        WAYPOINTS[8][2] = 3*pi/2
    if command == 'r26':
        WAYPOINTS[9][2] = pi/2

    
    # 8->1 and 1->8
    if command == 'r05':
        WAYPOINTS[1][2] = pi
    if command == 'r03':
        WAYPOINTS[8][2] = 0
    
    # 1->7 and 7->1
    if command == 'r02':
        WAYPOINTS[7][2] = pi
    if command == 'r04':
        WAYPOINTS[1][2] = 0
    
    # 3->5 and 5->3
    if command == 'r12':
        WAYPOINTS[5][2] = pi
    if command == 'r14':
        WAYPOINTS[3][2] = 3*pi/2
    


print(WAYPOINTS)
for ii in range(len(WAYPOINTS)):
    key_ = "wp" + str(ii)
    WAYPOINTS_DICT[key_] = WAYPOINTS[ii]
    print(WAYPOINTS_DICT[key_])


# 5) Program here the main commands of your mission planning algorithm for turtlebot ---------------------------------
""" Main code 
"""
if __name__ == '__main__':
    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        # input_t=input("")
        
        # 5.0) Testing the GNC module         
        # move_robot_waypoint0_waypoint1()
        # task6b()
     

	# 5.1) Starting the AI Planner
       #Here you must run your AI planner module
        import subprocess
        # python2.7 /home/ntnu-itk/catkin_ws/src/AI-planning/temporal-planning/bin/plan.py 
        # stp-2 
        # /home/ntnu-itk/catkin_ws/src/AI-planning/mydomain2.pddl 
        # /home/ntnu-itk/catkin_ws/src/AI-planning/myproblem2.pddl
        command = []

        # command = ["ls ", "~/catkin_ws/src/AI-planning/bin/"]
        command.append("source ~/catkin_ws/src/AI-planning/bin/activate; ")
        command.append("cd ~/catkin_ws/src/AI-planning/; ")
        command.append('python2.7 ')
        command.append("~/catkin_ws/src/AI-planning/temporal-planning/bin/plan.py ")
        command.append("stp-2 ")
        command.append("~/catkin_ws/src/AI-planning/mydomain2.pddl ")
        command.append("~/catkin_ws/src/AI-planning/myproblem2.pddl")
        tmp = ""
        for c_ in command:
            tmp = tmp + c_
        # print(tmp)
        result = subprocess.run(tmp, shell=True, capture_output=True, text=True)
        # print(result)
        # input("")
    
        # 5.2) Reading the plan 
        print("  ")
        print("Reading the plan from AI planner")
        print("  ")
        plan_general = read_plan()

        # 5.3) Start mission execution 
        # convert string into functions and executing
        print("")
        print("Starting mission execution")
        battery=100
        task_finished=0
        task_total=len(plan_general)
        i_ini=0

        while i_ini < task_total:
            # move_robot_waypoint0_waypoint1()
            #taking_photo_exe()

            # print(plan_general[i_ini])
            plan_temp=plan_general[i_ini].split(" ")
            print(plan_temp)
            
            if plan_temp[0]=="picture":
                print("Inspect -pump" + plan_temp[1])
                # taking_photo_exe()
                time.sleep(1)

            elif plan_temp[0]=="inspect":
                print("check-valve" + plan_temp[1])
                # taking_photo_exe()
                # Manipulate_OpenManipulator_x()
                time.sleep(1)
            
            elif plan_temp[0]=="move":
                print("move_robot_waypoints")
                map_direction(plan_temp[1])
                start_pos = WAYPOINTS[int(plan_temp[2][2:])]
                print(f"start_pos_index {int(plan_temp[2][2:])}")
                print(f"end_pos_index {int(plan_temp[3][2:])}")
                end_pos = WAYPOINTS[int(plan_temp[3][2:])]
                print(f"startpos {start_pos}")
                print(f"endpos {end_pos}")
                move_robot(start_pos=start_pos, end_pos=end_pos)
                # print(WAYPOINTS_DICT[plan_temp[2]])
                # print(WAYPOINTS_DICT[plan_temp[3]])
                # move_robot(WAYPOINTS_DICT[plan_temp[2]], WAYPOINTS_DICT[plan_temp[3]])
                
                time.sleep(1)
            
            elif plan_temp[0]=="charge":
                charge_battery_waypoint0()
            
            else:
                raise

            i_ini=i_ini+1  # Next tasks


        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(5)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
