from __future__ import division
import numpy as np

from car_controller.controller import BaseController
from car_controller.controller import compute_position_in_frame


class PIDController(BaseController):
    def __init__(self, **kwargs):
        self.kp = kwargs.pop("kp")
        self.ki = kwargs.pop("ki")
        self.kd = kwargs.pop("kd")

        # For integral control, need an accumulator
        self.error_sum = 0

        # Get the keyword args that we didn't consume with the above initialization
        super(PIDController, self).__init__(**kwargs)

    def reset_params(self, **kwargs):
        with self.state_lock:
            if not set(kwargs).issubset(set(self.__required)):
                raise ValueError(
                    "Invalid or incomplete set of keyword arguments provided", kwargs
                )
            # These next two lines set the instance attributes from the defaults and
            # kwargs dictionaries. For example, the key "kp" becomes the
            # instance attribute self.kp.
            self.__dict__.update(kwargs)

    def reset_state(self):
        super(PIDController, self).reset_state()
        with self.state_lock:
            self.error_sum = 0

    def get_error(self, pose, reference_xytv):
        """Compute the PID error.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed

        Returns:
            error: across-track and cross-track error
        """
        return compute_position_in_frame(pose, reference_xytv[:3])

    def get_control(self, pose, reference_xytv, error):
        """Compute the PID control law.

        Args:
            pose: current state of the vehicle [x, y, heading]
            reference_xytv: reference state and speed [x, y, heading, v]
            error: error vector from get_error [e_at, e_ct]

        Returns:
            control: np.array of velocity and steering angle
                (velocity should be copied from reference velocity)
        """
        # BEGIN SOLUTION "QUESTION 2.1" ALT="raise NotImplementedError"
        cross_track = error[1]
        
        heading_diff = pose[2] - reference_xytv[2]
        heading_diff = np.arctan2(np.sin(heading_diff), np.cos(heading_diff))
        
        self.error_sum += cross_track * (1.0 / self.frequency)
        
        ref_vel = reference_xytv[3]
        cross_track_derivative = ref_vel * np.sin(heading_diff)
        
        steering = -(self.kp * cross_track + self.ki * self.error_sum + self.kd * cross_track_derivative)
        
        return np.array([ref_vel, steering])

        # END SOLUTION