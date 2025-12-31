from .odrive_utils import ODriveUtils
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_IDLE,
    INPUT_MODE_VEL_RAMP,
)


class MotorController:
    def __init__(self, axis_index=0):
        """
        ODrive motor controller class.
        :param axis_index: 0 (left motor) or 1 (right motor)
        """
        self.axis_index = axis_index
        self.odrive = ODriveUtils.find_odrive()  # Find and connect to ODrive
        ODriveUtils.clear_odrive_errors(self.odrive)  # Clear errors on startup
        self.axis = self.select_axis()
        self.init_motor()

    def select_axis(self):
        """Select the ODrive axis based on the index."""
        if self.axis_index == 0:
            return self.odrive.axis0
        elif self.axis_index == 1:
            return self.odrive.axis1
        else:
            raise ValueError("Invalid axis_index! Use 0 or 1.")

    def init_motor(self):
        """Initialize motor: closed-loop control & ramped velocity mode."""
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.axis.controller.config.vel_ramp_rate = 10
        self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.axis.controller.config.pos_gain = 20
        self.axis.controller.config.vel_gain = 0.15
        self.axis.controller.config.vel_integrator_gain = 0.5
        self.axis.controller.config.vel_integrator_limit = 1
        print(
            f"Motor {self.axis_index}: Initialized in velocity control mode.",
            flush=True,
        )

    def get_velocity(self):
        """Get the current motor velocity [rps]."""
        vel = self.axis.encoder.vel_estimate
        # print(f"Motor {self.axis_index}: Current velocity = {vel:.2f} rps", flush=True)
        return vel

    def get_position(self):
        """Get the relative position (multi-turns) where the initial position is 0"""
        pos = self.axis.encoder.pos_estimate
        return pos


    def set_velocity(self, velocity):
        """Set target velocity [rps]."""
        try:
            self.axis.controller.input_vel = velocity
            # print(
            #     f"Motor {self.axis_index}: Velocity set to {velocity:.2f} rps.",
            #     flush=True,
            # )
        except Exception as e:
            print(f"Error setting velocity: {e}")

    def velfb_torque_control(self, vel_err, delta_vel, accum_vel_err):
        pass

    def stop(self):
        """Emergency stop: Set velocity to zero."""
        try:
            self.set_velocity(0.0)
            print(f"Motor {self.axis_index}: Emergency stop activated.")
        except Exception as e:
            print(f"Error during emergency stop: {e}")

    def set_idle(self):
        """Set the motor to idle (disable control)."""
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
            print(f"Motor {self.axis_index}: Set to idle mode.")
        except Exception as e:
            print(f"Error setting idle mode: {e}")

    def check_errors(self):
        """Check and print ODrive error status."""
        ODriveUtils.check_odrive_errors(self.odrive)

    def reboot(self):
        """Reboot the ODrive device."""
        ODriveUtils.reboot_odrive(self.odrive)
