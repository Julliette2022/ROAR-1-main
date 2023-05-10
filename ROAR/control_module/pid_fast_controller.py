from operator import truediv
from matplotlib.pyplot import close
from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle

from ROAR.utilities_module.data_structures_models import Transform, Location
from collections import deque
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path

class PIDFastController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        
        # useful variables
        self.old_pitch = 0
        self.delta_pitch = 0
        self.pitch_bypass = False
        self.force_brake = False

        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, most_recent_checkpoint, far_waypoint: Transform, **kwargs) -> VehicleControl:

        # run lat pid controller
        steering, error, wide_error, sharp_error = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)
        
        
        current_speed = Vehicle.get_speed(self.agent.vehicle)
        
        # get errors from lat pid
        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        #print(error, wide_error, sharp_error)

        # calculate change in pitch
        pitch = float(next_waypoint.record().split(",")[4])
        #print(next_waypoint.record())
        
        if pitch == 1.234567890:
            # bypass pitch
            self.pitch_bypass = True
        elif pitch == 0.987654321:
            # force break
            self.force_brake = True
        else:
            self.pitch_bypass = False
            self.force_brake = False
            self.delta_pitch = pitch - self.old_pitch
            self.old_pitch = pitch

        #print(pitch, self.delta_pitch)

        # throttle/brake control
        if self.force_brake:
            throttle = -1
            brake = 1
            #print("Force break")
        # Section 9 ramps
        elif current_speed > 260:
            throttle = -1
            brake = 1
        elif self.delta_pitch > 0.7 and current_speed > 100 and not self.pitch_bypass and most_recent_checkpoint in [9] and pitch > -1.0: # big ramp, high speed
            throttle = -1
            brake = 1
            # print("Big ramp high speed", pitch)
            #print(next_waypoint.record())
        elif most_recent_checkpoint == 6.5: # big ramp, high speed
            throttle = 0.75
            brake = 1
            # print("Big ramp high speed", pitch)
            #print(next_waypoint.record())
        elif sharp_error > 0.6 and current_speed > 85: # narrow turn
            throttle = -1
            brake = 1
            if most_recent_checkpoint in [14]:
                throttle = -0.5
                brake = 1
                steering = 1
            elif most_recent_checkpoint in [11]:
                brake = 0.5
                steering *= 0.7
            # print("Narrow turn")
        elif abs(steering) > 0.3 and current_speed > 45: # steering control
            throttle = 0.3
            brake = 0
            if most_recent_checkpoint == 5:
                throttle = 0.20
                brake = 0.2
            if most_recent_checkpoint in [14]:
                throttle = 1
                brake = 1
            elif most_recent_checkpoint in [11]:
                throttle *= 0.03
                brake = 0.5
            # print("Hard steering")
        elif wide_error > 0.05 and current_speed > 95: # wide turn
            throttle = max(0.2, 1 - 6.6*pow(wide_error + current_speed*0.0015, 3))
            brake = 0
            if most_recent_checkpoint not in [6, 8, 10, 12, 5, 9, 11]:
                throttle *= 1.2
            elif most_recent_checkpoint == 5:
                throttle *= 0.5
            elif most_recent_checkpoint in [14]:
                throttle = 1
                brake = 1
                steering = 1
            elif most_recent_checkpoint in [11]:
                throttle *= 0.05
                brake = 0.5
            # print("Wide turn")
        elif current_speed > self.max_speed:
            throttle = 0.7
            brake = 0.1
        else:
            throttle = 1
            brake = 0
        
        if most_recent_checkpoint == 12.25:
            throttle -= 0.5
            brake += 0.5

        # DEBUGGING
        #print(round(self.delta_pitch, 2))
        #print(round(wide_error, 2))
        
        # Setting gear
        #print(pitch)
        gear_quotient = 60
        gear = math.ceil((current_speed - (2*pitch)) / gear_quotient)
        if gear == 0:
            gear += 1
        if pitch > 3 and current_speed < 6: gear = 1
        print(self.agent.vehicle.transform.location.x)
        #print(self.agent.vehicle.transform.location.y)
        print(self.agent.vehicle.transform.location.z)
        print(self.agent.vehicle.transform.location)
        
        if ((self.agent.vehicle.transform.location.x >= 1300) and (self.agent.vehicle.transform.location.x <= 1400)) and ((self.agent.vehicle.transform.location.z >= 4000) and (self.agent.vehicle.transform.location.z <= 4400)) and ((self.agent.vehicle.transform.location.y >= 60) and (self.agent.vehicle.transform.location.y <= 80)) :
            brake = 0.03
            steering -= 0.01
        if ((self.agent.vehicle.transform.location.x >= 1300) and (self.agent.vehicle.transform.location.x <= 1400)) and ((self.agent.vehicle.transform.location.z >= 4000) and (self.agent.vehicle.transform.location.z <= 4400)) and ((self.agent.vehicle.transform.location.y >= 60) and (self.agent.vehicle.transform.location.y <= 80)) :
            brake = 0   
        
        if ((self.agent.vehicle.transform.location.x >= 1300) and (self.agent.vehicle.transform.location.x <= 1380)) and ((self.agent.vehicle.transform.location.z >= 3500) and (self.agent.vehicle.transform.location.z <= 3700)) and ((self.agent.vehicle.transform.location.y >= 60) and (self.agent.vehicle.transform.location.y <= 80)) :
            brake = 0.6
            throttle = 1
            #steering += 0.1
            
        if (self.agent.vehicle.transform.location.x >= 1300) and (self.agent.vehicle.transform.location.x <= 1350) and (self.agent.vehicle.transform.location.z >= 3590) and (self.agent.vehicle.transform.location.z <= 3700)  :
            brake = 1
            throttle = 0.5
          
        if (self.agent.vehicle.transform.location.x >= 3450) and (self.agent.vehicle.transform.location.x <= 3700) and (self.agent.vehicle.transform.location.z >= 3600) and (self.agent.vehicle.transform.location.z <= 3700) and (self.agent.vehicle.transform.location.y <= 190) and (self.agent.vehicle.transform.location.y <= 195) :
            brake = 1
            throttle = 0
            steering = 0
            
        if (self.agent.vehicle.transform.location.x >= 3700) and (self.agent.vehicle.transform.location.x <= 3800) and (self.agent.vehicle.transform.location.z >= 3300) and (self.agent.vehicle.transform.location.z <= 3400) :
            throttle = 0.5
            
        if (self.agent.vehicle.transform.location.x >= 3640) and (self.agent.vehicle.transform.location.x <= 3750) and (self.agent.vehicle.transform.location.z >= 2400) and (self.agent.vehicle.transform.location.z <= 2500) :
            throttle = 0.7  
        
        if (self.agent.vehicle.transform.location.x >= 3100) and (self.agent.vehicle.transform.location.x <= 3190) and (self.agent.vehicle.transform.location.z >= 2500) and (self.agent.vehicle.transform.location.z <= 2600) :
            #throttle = 1
            
            steering -= 0.2
            
        if (self.agent.vehicle.transform.location.x >= 3600) and (self.agent.vehicle.transform.location.x <= 3700) and (self.agent.vehicle.transform.location.z >= 2550) and (self.agent.vehicle.transform.location.z <= 2300) :
            throttle = 1
            
        if (self.agent.vehicle.transform.location.x >= 3600) and (self.agent.vehicle.transform.location.x <= 3700) and (self.agent.vehicle.transform.location.z >= 2400) and (self.agent.vehicle.transform.location.z <= 2500) :
            throttle = 1
            
        if (self.agent.vehicle.transform.location.x >= 3550) and (self.agent.vehicle.transform.location.x <= 3620) and (self.agent.vehicle.transform.location.z >= 2400) and (self.agent.vehicle.transform.location.z <= 2500) :
            brake = 0.7
            
        if (self.agent.vehicle.transform.location.x >= 3630) and (self.agent.vehicle.transform.location.x <= 3700) and (self.agent.vehicle.transform.location.z >= 2400) and (self.agent.vehicle.transform.location.z <= 2500) :
            throttle = 1
        
        if (self.agent.vehicle.transform.location.x >= 3650) and (self.agent.vehicle.transform.location.x <= 3720) and (self.agent.vehicle.transform.location.z >= 2400) and (self.agent.vehicle.transform.location.z <= 2500) :
            brake = 0
            throttle = 1
            
        if (self.agent.vehicle.transform.location.x >= 5200) and (self.agent.vehicle.transform.location.x <= 5400) and (self.agent.vehicle.transform.location.z >= 4000) and (self.agent.vehicle.transform.location.z <= 3800) :
            throttle = 0.6
            steering -= 0.4
            
            
        if (self.agent.vehicle.transform.location.x >= 4930) and (self.agent.vehicle.transform.location.x <= 4990) and (self.agent.vehicle.transform.location.z >= 3800) and (self.agent.vehicle.transform.location.z <= 3950) :
            throttle = 0.8
            brake = 0.4
            
        if (self.agent.vehicle.transform.location.x >= 4940) and (self.agent.vehicle.transform.location.x <= 5000) and (self.agent.vehicle.transform.location.z >= 3850) and (self.agent.vehicle.transform.location.z <= 3940) :
            throttle = 1
        
        if (self.agent.vehicle.transform.location.x >= 2450) and (self.agent.vehicle.transform.location.x <= 2550) and (self.agent.vehicle.transform.location.z >= 4200) and (self.agent.vehicle.transform.location.z <= 4300) :
            throttle = 0.8
            brake = 0.5
            steering +=0.2
            
        if (self.agent.vehicle.transform.location.x >= 2540) and (self.agent.vehicle.transform.location.x <= 2600) and (self.agent.vehicle.transform.location.z >= 4240) and (self.agent.vehicle.transform.location.z <= 4300) :
            brake = 1
            steering += 0.25
            
        if (self.agent.vehicle.transform.location.x >= 2560) and (self.agent.vehicle.transform.location.x <= 2600) and (self.agent.vehicle.transform.location.z >= 4230) and (self.agent.vehicle.transform.location.z <= 4260) :
            steering -= 0.3
            
        if (self.agent.vehicle.transform.location.x >= 2560) and (self.agent.vehicle.transform.location.x <= 2570) and (self.agent.vehicle.transform.location.z >= 4150) and (self.agent.vehicle.transform.location.z <= 4200) :
            steering += 0.1
            #brake = 0.4
        
        if (self.agent.vehicle.transform.location.x >= 2560) and (self.agent.vehicle.transform.location.x <= 2600) and (self.agent.vehicle.transform.location.z >= 4040) and (self.agent.vehicle.transform.location.z <= 4090) :
            brake = 0.8
        
        if (self.agent.vehicle.transform.location.x >= 5390) and (self.agent.vehicle.transform.location.x <= 5450) and (self.agent.vehicle.transform.location.z >= 3700) and (self.agent.vehicle.transform.location.z <= 3760) :
            throttle = 0.6
        
        if (self.agent.vehicle.transform.location.x >= 2550) and (self.agent.vehicle.transform.location.x <= 2600) and (self.agent.vehicle.transform.location.z >= 4100) and (self.agent.vehicle.transform.location.z <= 4200) :
            
            throttle = 0.8
        
        #if (self.agent.vehicle.transform.location.x >= 2510) and (self.agent.vehicle.transform.location.x <= 2580) and (self.agent.vehicle.transform.location.z >= 4160) and (self.agent.vehicle.transform.location.z <= 4220) :
            #brake = 0.9
            #throttle = 0
            #steering -= -1
        #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        

        if (self.agent.vehicle.transform.location.x >= 2570) and (self.agent.vehicle.transform.location.x <= 2590) and (self.agent.vehicle.transform.location.z >= 4250) and (self.agent.vehicle.transform.location.z <= 4360) :
            brake = 1

        if (self.agent.vehicle.transform.location.x >= 2570) and (self.agent.vehicle.transform.location.x <= 2580) and (self.agent.vehicle.transform.location.z >= 4230) and (self.agent.vehicle.transform.location.z <= 4340) :
            steering += 0.2

        # 点11 环形路口
    
        if (self.agent.vehicle.transform.location.x >= 5270) and (self.agent.vehicle.transform.location.x <= 5350) and (self.agent.vehicle.transform.location.z >= 3940) and (self.agent.vehicle.transform.location.z <= 3950) :
            steering -= 0.2
            brake = 0.7
        # 点9 大坡起伏

        if (self.agent.vehicle.transform.location.x >= 1970) and (self.agent.vehicle.transform.location.x <= 2155) and (self.agent.vehicle.transform.location.z >= 3415) and (self.agent.vehicle.transform.location.z <= 3440) :
            brake = 1
            throttle = 0.7
        # 点2 弯道

        

        return VehicleControl(throttle=throttle, steering=steering, brake=brake, manual_gear_shift=True, gear=gear)

    
        
    @staticmethod
    def find_k_values(vehicle: Vehicle, config: dict) -> np.array:
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])

class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #error = np.arccos(v_vec_normed @ w_vec_normed.T)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive
        _cross = np.cross(v_vec_normed, w_vec_normed)

        # calculate close error projection
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        wide_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        # calculate far error projection
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        #sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        sharp_error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1)) # makes sure arccos input is between -1 and 1, inclusive

        if _cross[1] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = PIDFastController.find_k_values(config=self.config, vehicle=self.agent.vehicle)
        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )
        return lat_control, error, wide_error, sharp_error
