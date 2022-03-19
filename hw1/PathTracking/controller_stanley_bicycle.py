import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, kp=0.5):
        self.path = None
        self.kp = kp

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
        x, y, yaw, delta, v, l = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"]

        # Search Front Wheel Target
        front_x = x + l*np.cos(np.deg2rad(yaw))
        front_y = y + l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta))
        min_idx, min_dist = utils.search_nearest(self.path, (front_x,front_y))
        target = self.path[min_idx]

        # TODO: Stanley Control for Bicycle Kinematic Model
        yaw_e = target[2] - yaw
        yaw_e = utils.angle_norm(yaw_e)
        # e_hat = vf*np.sin(np.deg2rad(delta - yaw_e))
        term1 = np.cos(np.deg2rad(target[2]+90))
        term2 = np.sin(np.deg2rad(target[2]+90))
        e = np.dot((x-target[0],y-target[1]), (term1,term2))
        next_delta = np.rad2deg(np.arctan2((-1*self.kp*e), vf)) + yaw_e

        return next_delta, target
