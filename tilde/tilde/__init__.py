#!/usr/bin/env python3

from rclpy.executors import MultiThreadedExecutor
from math import radians, inf, isnan, pi, cos, sin, sqrt
import random
from random import random, uniform
import csv
from datetime import datetime
import os

import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions
import time

LASER_RANGE = 3
LASER_FREQ = 10

STOP_WALL_LEN = 1.95
STOP_TOLERANCE = 0.1
STOP_MIN = STOP_WALL_LEN * (1 - STOP_TOLERANCE)
STOP_MAX = STOP_WALL_LEN * (1 + STOP_TOLERANCE)

RAD22_5 = radians(22.5)
RAD45 = radians(45)
RAD67_5 = radians(67.5)
RAD90 = radians(90)
RAD112_5 = radians(112.5)
RAD135 = radians(135)
RAD157_5 = radians(157.5)

# Experiment configurations
EXPERIMENTS = [
    {
        "name": "baseline",
        "robot1": {
            "maxLinVel": 1.1,
            "maxAngVel": 3.0,
            "linAcc": 1.2,
            "linDec": 3.0,
            "angAcc": 3.0,
            "angDec": 3.0,
        },
        "robot2": {
            "maxLinVel": 2.2,  
            "maxAngVel": 3.0,
            "linAcc": 1.5,
            "linDec": 3.0,
            "angAcc": 3.0,
            "angDec": 3.0,
        },
    },
    {
        "name": "faster in corners",
        "robot1": {
            "maxLinVel": 1.1,
            "maxAngVel": 3.0,
            "linAcc": 1.2,
            "linDec": 3.0,
            "angAcc": 6.0,
            "angDec": 6.0,
        },
        "robot2": {
            "maxLinVel": 2.2,  
            "maxAngVel": 3.0,  
            "linAcc": 1.5,
            "linDec": 3.0,
            "angAcc": 6.0,
            "angDec": 6.0, 
        },
    },
    {
        "name": "faster in corners and in general",
        "robot1": {
            "maxLinVel": 1.1,
            "maxAngVel": 3.0,
            "linAcc": 1.5,
            "linDec": 3.0,
            "angAcc": 6.0,
            "angDec": 6.0,
        },
        "robot2": {
            "maxLinVel": 2.2,  
            "maxAngVel": 3.0,  
            "linAcc": 3.0,
            "linDec": 3.0,
            "angAcc": 6.0,
            "angDec": 6.0, 
        },
    },
    {
        "name": "aggressive_pursuit",
        "robot1": {
            "maxLinVel": 1.1,
            "maxAngVel": 3.0,
            "linAcc": 1.5,
            "linDec": 3.0,
            "angAcc": 12.0,
            "angDec": 12.0,
        },
        "robot2": {
            "maxLinVel": 2.2,  
            "maxAngVel": 3.0,  
            "linAcc": 3.0,
            "linDec": 3.0,
            "angAcc": 12.0,
            "angDec": 12.0,

            #leder often escapes fast robots radar
        },
        "laser_freq": 10,  # Highest sensor rate
    },

        {
        "name": "aggressive_pursuit,better lasers",
        "robot1": {
            "maxLinVel": 1.1,
            "maxAngVel": 3.0,
            "linAcc": 1.5,
            "linDec": 3.0,
            "angAcc": 12.0,
            "angDec": 12.0,
        },
        "robot2": {
            "maxLinVel": 2.2,  
            "maxAngVel": 3.0,  
            "linAcc": 3.0,
            "linDec": 3.0,
            "angAcc": 20.0,
            "angDec": 20.0,

        },
        "laser_freq": 100,  # Highest sensor rate
    },
]


class PathRecorder:
    def __init__(self, experiment_name, robot_name):
        self.experiment_name = experiment_name
        self.robot_name = robot_name
        self.path_data = []
        self.count = 0
        
        self.data_dir = f"experiment_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.data_dir, exist_ok=True)
        
    def record_position(self, seq, timestamp, x, y):
        self.count += 1
        self.path_data.append({
            'seq': self.count,
            'timestamp': timestamp,
            'x': x,
            'y': y
        })
        
    def save_path(self):
        filename = f"{self.data_dir}/{self.experiment_name}_{self.robot_name}_path.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['seq', 'timestamp', 'x', 'y'])
            writer.writeheader()
            writer.writerows(self.path_data)


def clamp(val, minVal, maxVal):
    return float(max(min(val, maxVal), minVal))


def pointDist(p1, p2) -> float:
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def laserToPoint(laser):
    return (laser[1] * cos(laser[0]), laser[1] * sin(laser[0]))


class SerpController(Node):
    doStop = False
    doOdometry = True


    minDistFromWall = 1.0
    k = 3

    target_position = [-5, -4.5]
    position_tolerance = 1
    experiment_complete = False
    

    def __init__(self, experiment_config) -> None:
        super().__init__("SerpController")
        

                # Apply experiment configuration
        self.maxLinVel = experiment_config["robot1"]["maxLinVel"]
        self.maxAngVel = experiment_config["robot1"]["maxAngVel"]
        self.linAcc = experiment_config["robot1"]["linAcc"]
        self.linDec = experiment_config["robot1"]["linDec"]
        self.angAcc = experiment_config["robot1"]["angAcc"]
        self.angDec = experiment_config["robot1"]["angDec"]
        self.count = 50
        
        self.path_recorder = PathRecorder(experiment_config["name"], "robot1")
        

        self.vel = Twist()

        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        if SerpController.doOdometry:
            print('"seq","sec","x","y"')
            self.create_subscription(
                Odometry, "odom", self._odometryGroundTruth, 1
            )
            
        self.create_subscription(LaserScan, "/static_laser", self._scanCallback, 1)



    def _odometryGroundTruth(self, odometry):
        timestamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        if self.count > 0:
            self.count -=1

        self.path_recorder.record_position(None, timestamp, x, y)

        if not SerpController.experiment_complete and self.count == 0:
            if abs(x - SerpController.target_position[0]) <= SerpController.position_tolerance and \
               abs(y - SerpController.target_position[1]) <= SerpController.position_tolerance:
                SerpController.experiment_complete = True
                self.get_logger().info("Target position reached. Ending experiment.")

    def _scanCallback(self, scan):
        lasers = [0] * len(scan.ranges)
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            dist = scan.ranges[i]
            lasers[i] = (angle, dist if not isnan(dist) else inf)
            angle += scan.angle_increment

        dirs = self._processLasers(lasers, scan.angle_increment)
        self.reactToScan(dirs)

    def _processLasers(self, lasers, angleIncrement):
        angleMin = lasers[0][0]

        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": RAD22_5 * 2},
            "front_right": {"dist": inf, "ang": -RAD22_5 * 2},
            "left": {"dist": inf, "ang": RAD22_5 * 4},
            "right": {"dist": inf, "ang": -RAD22_5 * 4},
            "back_left": {"dist": inf, "ang": RAD22_5 * 6},
            "back_right": {"dist": inf, "ang": -RAD22_5 * 6},
            "back": {"dist": inf, "ang": pi},
            "minDir": "front",
            "edges": {
                "left": inf,
                "front_left": inf,
                "back_left": inf,
            },
        }

        dirs["edges"]["left"] = self._calcWallLen(
            lasers, int((RAD90 - angleMin) / angleIncrement)
        )
        dirs["edges"]["right"] = self._calcWallLen(
            lasers, int((-RAD90 - angleMin) / angleIncrement)
        )

        minDist = inf
        for laser in lasers:
            angle = laser[0]
            dist = laser[1]

            absAngle = abs(angle)
            if absAngle <= RAD22_5:
                key = "front"
            elif absAngle <= RAD67_5:
                key = "front_left" if angle > 0 else "front_right"
            elif absAngle <= RAD112_5:
                key = "left" if angle > 0 else "right"
            elif absAngle <= RAD157_5:
                key = "back_left" if angle > 0 else "back_right"
            else:
                key = "back"

            if abs(angle - RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_left"] = dist
            elif abs(angle - -RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_right"] = dist
            elif abs(angle - RAD135) < angleIncrement * 2:
                dirs["edges"]["back_left"] = dist
            elif abs(angle - -RAD135) < angleIncrement * 2:
                dirs["edges"]["back_right"] = dist

            if dist < dirs[key]["dist"]:
                # update sector
                dirs[key]["dist"] = dist
                dirs[key]["ang"] = angle
                if dist < minDist:
                    # update global info
                    dirs["minDir"] = key
                    minDist = dist

        return dirs

    def _calcWallLen(self, lasers, idx):
        if lasers[idx][1] != inf:
            firstIdx = lastIdx = idx
            i = 1
            while idx + i < len(lasers):
                newLastIdx = idx + i
                if lasers[newLastIdx][1] != inf:
                    lastIdx = newLastIdx
                i += 1
            i = 1
            while idx - i >= 0:
                newFirstIdx = idx - i
                if lasers[newFirstIdx][1] != inf:
                    firstIdx = newFirstIdx
                i += 1

            ret = 0
            points = map(
                lambda idx: laserToPoint(lasers[idx]), range(firstIdx, lastIdx + 1)
            )
            try:
                p1 = next(points)
                while True:
                    p2 = next(points)
                    ret += pointDist(p1, p2)
                    p1 = p2
            except StopIteration:
                pass
            return ret
        else:
            return inf

    def reactToScan(self, dirs):
        minDir = dirs["minDir"]
        minDist = dirs[minDir]["dist"]
        minAng = dirs[minDir]["ang"]

        if minDist == inf:
            # can't see anything => random walk
            self.wiggle()
            return

        if SerpController.doStop and abs(minDist - SerpController.minDistFromWall) < 0.1 * SerpController.k:
            if (
                dirs["edges"]["back_left"] == inf
                and STOP_MIN < dirs["edges"]["left"] < STOP_MAX
            ):
                self.get_logger().info(f"End left:  {dirs['edges']['left']}")
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return
            elif (
                dirs["edges"]["back_right"] == inf
                and STOP_MIN < dirs["edges"]["right"] < STOP_MAX
            ):
                self.get_logger().info(f"End right: {dirs['edges']['right']}")
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return

        if minDir.endswith("right"):
            front = min(dirs["front"]["dist"], dirs["front_left"]["dist"])
        elif minDir.endswith("left"):
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"])
        else:
            front = min(
                dirs["front"]["dist"],
                dirs["front_right"]["dist"],
                dirs["front_left"]["dist"],
            )
        self.linVel = self.maxLinVel * front / LASER_RANGE

        if minDir == "front":
            if dirs["left"]["dist"] == inf and dirs["right"]["dist"] != inf:
                # was following wall on right and found obstacle in front => circle obstacle
                wallSide = "right"
            elif dirs["left"]["dist"] != inf and dirs["right"]["dist"] == inf:
                # was following wall on left and found obstacle in front => circle obstacle
                wallSide = "left"
            else:
                # found obstacle in front, which way to turn? The closest
                minLeft = min(dirs["front_left"]["dist"], dirs["left"]["dist"])
                minRight = min(dirs["front_right"]["dist"], dirs["right"]["dist"])
                wallSide = "left" if minLeft < minRight else "right"
        elif minDir.endswith("left"):
            # keep following left
            wallSide = "left"
        else:
            # keep following right
            wallSide = "right"

        if wallSide == "left":
            angDistTerm = cos(minAng) + (SerpController.minDistFromWall - minDist)
        else:
            angDistTerm = cos(pi - minAng) + (minDist - SerpController.minDistFromWall)
        self.angVel = -SerpController.k * self.linVel * angDistTerm

        self.moveTurtle()

    @property
    def linVel(self):
        return self.vel.linear.x

    @linVel.setter
    def linVel(self, newLinVel):
        desiredVel = clamp(newLinVel, -self.maxLinVel, self.maxLinVel)

        # v = v0 + a * t
        # desiredVel = self.linVel + a * (1/LASER_FREQ)
        a = (desiredVel - self.linVel) * LASER_FREQ
        if a > 0:
            if a <= self.linAcc:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max acceleration
                self.vel.linear.x = self.linVel + self.linAcc / LASER_FREQ
        elif a < 0:
            if a >= self.linDec:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max decelaration
                self.vel.linear.x = self.linVel - self.linDec / LASER_FREQ

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        desiredVel = clamp(newAngVel, -self.maxAngVel, self.maxAngVel)
        

        a = (desiredVel - self.angVel) * LASER_FREQ
        if a > 0:
            if a <= self.angAcc:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max acceleration
                self.vel.angular.z = self.angVel + self.angAcc / LASER_FREQ
        elif a < 0:
            if a >= self.angDec:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max decelaration
                self.vel.angular.z = self.angVel - self.angDec / LASER_FREQ

    def wiggle(self):
        #  rospy.loginfo("wiggling")
        self.linVel = self.maxLinVel
        v = self.maxAngVel * random()
        if random() > 0.5:
            self.angVel += v
        else:
            self.angVel -= v
        # reset wandering angle when limit reached
        if abs(self.angVel) >= self.maxAngVel:
            self.angVel = 0

        self.moveTurtle()

    def moveTurtle(self):
        self.pub.publish(self.vel)

    def reset(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.pub.publish(vel)

        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = "SerpController"
        request.pose = Pose2D(uniform(-6, 6), uniform(-5, 7), uniform(0, 359))
        client.call(request)

class SerpController2(Node):
    doStop = False
    doOdometry = True

    minDistFromWall = 1.0
    k = 3
    follow_distance_threshold = 1.5  # Distance threshold for detecting the other robot
    wall_follow_threshold = 3.0      # Distance threshold to fallback to wall following
    
    # Add new safety parameters
    min_safe_distance = 0.8  # Minimum safe distance to maintain from leader
    optimal_follow_distance = 1.2 


    def __init__(self, experiment_config) -> None:
        super().__init__("SerpController2")


                # Apply experiment configuration
        self.maxLinVel = experiment_config["robot2"]["maxLinVel"]
        self.maxAngVel = experiment_config["robot2"]["maxAngVel"]
        self.linAcc = experiment_config["robot2"]["linAcc"]
        self.linDec = experiment_config["robot2"]["linDec"]
        self.angAcc = experiment_config["robot2"]["angAcc"]
        self.angDec = experiment_config["robot2"]["angDec"]
        
        # Initialize path recorder
        self.path_recorder = PathRecorder(experiment_config["name"], "robot2")
        

        self.robot_detected = False
        self.metting = False
        self.vel = Twist()
        self.pub: Publisher = self.create_publisher(Twist, "/robot2/cmd_vel", 1)

        if SerpController2.doOdometry:
            print('"seq","sec","x","y"')
            self.create_subscription(
                Odometry, "robot2/odom", self._odometryGroundTruth, 1
            )

        self.create_subscription(LaserScan, "/robot2/static_laser", self._scanCallback, 1)
        self.create_subscription(LaserScan, "/robot2/robot_scan", self._robotScanCallback, 1)


    def _odometryGroundTruth(self, odometry):
        timestamp = odometry.header.stamp.sec + odometry.header.stamp.nanosec / 1e9
        
        if self.metting ==  False and self.robot_detected ==True:
            self.metting = True
            self.get_logger().info("!!!METTING!!! : " + str(timestamp)) 

        self.path_recorder.record_position(
            None,  # No sequence number available
            timestamp,
            odometry.pose.pose.position.x,
            odometry.pose.pose.position.y
        )

    def _scanCallback(self, scan):
        # Wall-following only if robot is not detected
        if not self.robot_detected:
            #self.get_logger().info("Executing wall-following behavior")
            lasers = [0] * len(scan.ranges)
            angle = scan.angle_min
            for i in range(len(scan.ranges)):
                dist = scan.ranges[i]
                lasers[i] = (angle, dist if not isnan(dist) else inf)
                angle += scan.angle_increment

            dirs = self._processLasers(lasers, scan.angle_increment)
            self.reactToScan(dirs)



    def _robotScanCallback(self, scan):
        lasers = [0] * len(scan.ranges)
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            dist = scan.ranges[i]
            lasers[i] = (angle, dist if not isnan(dist) else inf)
            angle += scan.angle_increment

        dirs = self._processLasers(lasers, scan.angle_increment)
        
        # Check if robot is detected in front
        if dirs["front"]["dist"] < self.wall_follow_threshold:
            if not self.robot_detected and dirs["front"]["dist"] < self.follow_distance_threshold:
                self.robot_detected = True
                self.get_logger().info("Robot detected, switching to following mode")
            
            if self.robot_detected:
                # Use enhanced following with safety distance
                self.follow_robot(dirs["front"]["dist"], dirs["front"]["ang"])
        else:
            if self.robot_detected:
                self.robot_detected = False
                self.get_logger().info("Leader lost, switching to wall mode")
                # Reset to wall-following parameters
                #self.maxLinVel = 1.1
                self.linAcc = 0.2
                #self.follow_distance_threshold = 2.5
                

    def follow_robot(self, distance, angle):
        # Implement dynamic speed control based on distance
        if distance < self.min_safe_distance:
            # Too close - stop completely
            self.linVel = 0
            self.angVel = angle * 0.5
            self.get_logger().warn(f"Too close to leader! Stopping. Distance: {distance}")
        
        elif distance < self.optimal_follow_distance:
            # Close to optimal distance - move very slowly
            speed_factor = (distance - self.min_safe_distance) / (self.optimal_follow_distance - self.min_safe_distance)
            self.linVel = min(self.maxLinVel * 0.3, speed_factor * 0.5)
            self.angVel = angle * 0.5
            self.get_logger().info(f"Maintaining safe distance. Speed: {self.linVel}")
        
        else:
            # Normal following behavior with smooth speed transition
            distance_error = distance - self.optimal_follow_distance
            speed_factor = min(1.0, distance_error / 2.0)  # Gradually increase speed
            self.linVel = min(self.maxLinVel, speed_factor * self.maxLinVel)
            self.angVel = angle * 0.5
            self.get_logger().info(f"Following leader. Distance: {distance}, Speed: {self.linVel}")

        self.moveTurtle()


    def _processLasers(self, lasers, angleIncrement):
        # Existing laser processing logic remains unchanged
        angleMin = lasers[0][0]

        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": RAD22_5 * 2},
            "front_right": {"dist": inf, "ang": -RAD22_5 * 2},
            "left": {"dist": inf, "ang": RAD22_5 * 4},
            "right": {"dist": inf, "ang": -RAD22_5 * 4},
            "back_left": {"dist": inf, "ang": RAD22_5 * 6},
            "back_right": {"dist": inf, "ang": -RAD22_5 * 6},
            "back": {"dist": inf, "ang": pi},
            "minDir": "front",
            "edges": {
                "left": inf,
                "front_left": inf,
                "back_left": inf,
            },
        }

        dirs["edges"]["left"] = self._calcWallLen(
            lasers, int((RAD90 - angleMin) / angleIncrement)
        )
        dirs["edges"]["right"] = self._calcWallLen(
            lasers, int((-RAD90 - angleMin) / angleIncrement)
        )

        minDist = inf
        for laser in lasers:
            angle = laser[0]
            dist = laser[1]

            absAngle = abs(angle)
            if absAngle <= RAD22_5:
                key = "front"
            elif absAngle <= RAD67_5:
                key = "front_left" if angle > 0 else "front_right"
            elif absAngle <= RAD112_5:
                key = "left" if angle > 0 else "right"
            elif absAngle <= RAD157_5:
                key = "back_left" if angle > 0 else "back_right"
            else:
                key = "back"

            if abs(angle - RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_left"] = dist
            elif abs(angle - -RAD22_5) < angleIncrement * 2:
                dirs["edges"]["front_right"] = dist
            elif abs(angle - RAD135) < angleIncrement * 2:
                dirs["edges"]["back_left"] = dist
            elif abs(angle - -RAD135) < angleIncrement * 2:
                dirs["edges"]["back_right"] = dist

            if dist < dirs[key]["dist"]:
                # Update sector
                dirs[key]["dist"] = dist
                dirs[key]["ang"] = angle
                if dist < minDist:
                    # Update global info
                    dirs["minDir"] = key
                    minDist = dist

        return dirs

    def _calcWallLen(self, lasers, idx):
        if lasers[idx][1] != inf:
            firstIdx = lastIdx = idx
            i = 1
            while idx + i < len(lasers):
                newLastIdx = idx + i
                if lasers[newLastIdx][1] != inf:
                    lastIdx = newLastIdx
                i += 1
            i = 1
            while idx - i >= 0:
                newFirstIdx = idx - i
                if lasers[newFirstIdx][1] != inf:
                    firstIdx = newFirstIdx
                i += 1

            ret = 0
            points = map(
                lambda idx: laserToPoint(lasers[idx]), range(firstIdx, lastIdx + 1)
            )
            try:
                p1 = next(points)
                while True:
                    p2 = next(points)
                    ret += pointDist(p1, p2)
                    p1 = p2
            except StopIteration:
                pass
            return ret
        else:
            return inf

    def reactToScan(self, dirs):
        # Wall-following logic remains active only if robot_detected is False
        if self.robot_detected:
            return  # Skip wall-following logic when in following mode

        minDir = dirs["minDir"]
        minDist = dirs[minDir]["dist"]
        minAng = dirs[minDir]["ang"]

        if minDist == inf:
            # can't see anything => random walk
            self.wiggle()
            return

        if SerpController2.doStop and abs(minDist - SerpController2.minDistFromWall) < 0.1 * SerpController2.k:
            if dirs["edges"]["back_left"] == inf and STOP_MIN < dirs["edges"]["left"] < STOP_MAX:
                self.get_logger().info(f"End left:  {dirs['edges']['left']}")
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return
            elif dirs["edges"]["back_right"] == inf and STOP_MIN < dirs["edges"]["right"] < STOP_MAX:
                self.get_logger().info(f"End right: {dirs['edges']['right']}")
                self.linVel = 0
                self.angVel = 0
                self.moveTurtle()
                return

        if minDir.endswith("right"):
            front = min(dirs["front"]["dist"], dirs["front_left"]["dist"])
        elif minDir.endswith("left"):
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"])
        else:
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"], dirs["front_left"]["dist"])

        self.linVel = self.maxLinVel * front / LASER_RANGE

        wallSide = "left" if minDir.endswith("left") else "right"
        angDistTerm = (
            cos(minAng) + (SerpController2.minDistFromWall - minDist)
            if wallSide == "left"
            else cos(pi - minAng) + (minDist - SerpController2.minDistFromWall)
        )
        self.angVel = -SerpController2.k * self.linVel * angDistTerm

        self.moveTurtle()

    @property
    def linVel(self):
        return self.vel.linear.x

    @linVel.setter
    def linVel(self, newLinVel):
        desiredVel = clamp(newLinVel, -self.maxLinVel, self.maxLinVel)

        # v = v0 + a * t
        # desiredVel = self.linVel + a * (1/LASER_FREQ)
        a = (desiredVel - self.linVel) * LASER_FREQ
        if a > 0:
            if a <= self.linAcc:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max acceleration
                self.vel.linear.x = self.linVel + self.linAcc / LASER_FREQ
        elif a < 0:
            if a >= self.linDec:
                self.vel.linear.x = desiredVel
            else:
                # exceeded max decelaration
                self.vel.linear.x = self.linVel - self.linDec / LASER_FREQ

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        desiredVel = clamp(newAngVel, -self.maxAngVel, self.maxAngVel)

        a = (desiredVel - self.angVel) * LASER_FREQ
        if a > 0:
            if a <= self.angAcc:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max acceleration
                self.vel.angular.z = self.angVel + self.angAcc / LASER_FREQ
        elif a < 0:
            if a >= self.angDec:
                self.vel.angular.z = desiredVel
            else:
                # exceeded max decelaration
                self.vel.angular.z = self.angVel - self.angDec / LASER_FREQ


    def wiggle(self):
        #  rospy.loginfo("wiggling")
        self.linVel = self.maxLinVel
        v = self.maxAngVel * random()
        if random() > 0.5:
            self.angVel += v
        else:
            self.angVel -= v
        # reset wandering angle when limit reached
        if abs(self.angVel) >= self.maxAngVel:
            self.angVel = 0
        self.get_logger().info(f"Wiggling: linVel={self.linVel}, angVel={self.angVel}")
      
        self.moveTurtle()

    def moveTurtle(self):
        # Publish current velocity
        #self.get_logger().info(f"Moving: linear={self.vel.linear.x}, angular={self.vel.angular.z}")
        self.pub.publish(self.vel)

    def reset(self):
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.pub.publish(vel)

        client = self.create_client(MoveModel, "/robot2/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = "SerpController2"
        request.pose = Pose2D(uniform(-6, 6), uniform(-5, 7), uniform(0, 359))
        client.call(request)



def run_experiment(experiment_config):
    rclpy.init()

    serp = SerpController(experiment_config)
    serp2 = SerpController2(experiment_config)

    executor = MultiThreadedExecutor()
    executor.add_node(serp)
    executor.add_node(serp2)

    try:
        while not SerpController.experiment_complete:
            executor.spin_once(timeout_sec=1.0)
    finally:
        # Save data and clean up when the experiment is complete
        serp.path_recorder.save_path()
        serp2.path_recorder.save_path()
        print(f"Experiment '{experiment_config['name']}' completed. Path data saved.")
        
        executor.shutdown()
        serp.destroy_node()
        serp2.destroy_node()
        rclpy.shutdown()




def main(args = None):
    # rclpy.init()
    
    # serp = SerpController()
    # serp2 = SerpController2()

    # rclpy.spin(serp)
    # rclpy.spin(serp2)
    experimentB = True

    
    
    experiment = EXPERIMENTS[5]
    print(f"Starting experiment: {experiment['name']}")
    run_experiment(experiment)
    print(f"Experiment {experiment['name']} completed")

    """
    else: 
        rclpy.init(args=args)
        
        serp = SerpController()
        serp2 = SerpController2()

        # Use a MultiThreadedExecutor to manage multiple nodes
        executor = MultiThreadedExecutor()
        executor.add_node(serp)
        executor.add_node(serp2)
        try:
            executor.spin()
        finally:
            # Cleanup
            executor.shutdown()
            serp.destroy_node()
            serp2.destroy_node()
            rclpy.shutdown()
            """

if __name__ == "__main__":
    main()
