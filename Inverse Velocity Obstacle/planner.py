#!/usr/bin/env

from __future__ import print_function
import time
import math
import threading

import numpy as np
from drone import BebopController as Drone
import matlab
import matlab.engine

class RosPlanner():
    def __init__(self, safe_dist, goal, drone, obs_buffer_size=3):
        self.safe_radius = safe_dist
        self.goal = goal
        self.drone = drone
        self.obs_buffer_size = obs_buffer_size
        self.obstacles = {}
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.stop = False
        self.matlab = matlab.engine.start_matlab()
        _ = self.matlab.addpath('/home/.mrd/Playground/Velocity-Obstacle/src/navigator/scripts')
    
    def obstacle_tracker(self, tags):
        for tag in tags.detections:
            if tag.pose.pose.position.x < self.safe_radius:
                pos = np.array([
                    tag.pose.pose.position.x,
                    tag.pose.pose.position.z
                ])
                try:
                    self.obstacles[tag.id].append(pos)
                    self.obstacles[tag.id][0] += 1
                    del self.obstacles[tag.id][1]
                except Exception:
                    self.obstacles[tag.id] = [1]
                    for i in range(self.obs_buffer_size):
                        self.obstacles[tag.id].append(pos)

    def odometry(self, odom):
        self.position = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y
        ])
        self.velocity = np.array([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y
        ])
    
    def __get_obs_states(self):
        obs_vels = {}
        obs_pos = {}
        for id in self.obstacles.keys():
            velocity = 0
            if self.obstacles[id][0] >= self.obs_buffer_size:
                velocities = []
                for i in range(self.obs_buffer_size-1):
                    tmp_vel = self.obstacles[id][i+1] - self.obstacles[id][i]
                    velocities.append(tmp_vel)
                
                factor = 1
                norm_weights = 0
                for i in range(self.obs_buffer_size-1):
                    velocity += factor*velocities[-i]
                    norm_weights += factor
                    factor *= 0.5
                velocity /= norm_weights
            obs_vels[id] = matlab.double(velocity.tolist())
            obs_pos[id] = matlab.double(self.obstacles[id][-1].tolist())
        return obs_pos.values(), obs_vels.values()

    def __goal_reached(self):
        error = ((self.goal-self.position) ** 2).sum()
        if error < 1:
            return True
        return False

    def __cap_velocity(self, velocity):
        velocity[0] = max(-2,velocity[0])
        velocity[0] = min(2,velocity[0])
        velocity[1] = max(-2,velocity[1])
        velocity[1] = min(2,velocity[1])
        return velocity

    def run(self):
        while not (self.__goal_reached() or self.stop):
            obs_positions, obs_velocities = self.__get_obs_states()
            delta_controls = self.matlab.getControls(
                matlab.double([0,0]),
                matlab.double([0,0]),
                matlab.double(obs_positions),
                matlab.double(obs_velocities),
                matlab.double(self.goal.tolist()),
                1,
            )
            delta_controls = np.array([delta_controls[0][0], delta_controls[1][0]])
            controls = self.__cap_velocity(self.velocity + delta_controls)
            print(controls)
            self.drone.move_by_velocity(controls)
