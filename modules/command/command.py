"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return True, cls(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.logger = local_logger

        # Constants
        self.height_tolerance = 0.5  # meters
        self.angle_tolerance = 5  # degrees
        self.z_speed = 1  # m/s
        self.turning_speed = 5  # deg/s

        # For average velocity calculation
        self.total_velocity_x = 0.0
        self.total_velocity_y = 0.0
        self.total_velocity_z = 0.0
        self.velocity_count = 0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData,
    ) -> "str | None":
        """
        Make a decision based on received telemetry data.
        """
        # Update average velocity
        if telemetry_data.x_velocity is not None:
            self.total_velocity_x += telemetry_data.x_velocity
        if telemetry_data.y_velocity is not None:
            self.total_velocity_y += telemetry_data.y_velocity
        if telemetry_data.z_velocity is not None:
            self.total_velocity_z += telemetry_data.z_velocity
        self.velocity_count += 1

        # Log average velocity for this trip so far
        avg_vx = self.total_velocity_x / self.velocity_count
        avg_vy = self.total_velocity_y / self.velocity_count
        avg_vz = self.total_velocity_z / self.velocity_count
        self.logger.info(f"Average velocity: ({avg_vx}, {avg_vy}, {avg_vz})")

        # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
        # The appropriate commands to use are instructed below

        # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
        # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"
        if telemetry_data.z is not None:
            delta_z = self.target.z - telemetry_data.z
            if abs(delta_z) > self.height_tolerance:
                # Send COMMAND_LONG with MAV_CMD_CONDITION_CHANGE_ALT
                self.connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                    0,  # confirmation
                    self.z_speed,  # param1: descent/ascent rate
                    0,  # param2: empty
                    0,  # param3: empty
                    0,  # param4: empty
                    0,  # param5: empty
                    0,  # param6: empty
                    self.target.z,  # param7: target altitude
                )
                return f"CHANGE_ALTITUDE: {delta_z}"

        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
        # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
        # Positive angle is counter-clockwise as in a right handed system
        if (
            telemetry_data.x is not None
            and telemetry_data.y is not None
            and telemetry_data.yaw is not None
        ):
            # Calculate angle to target from current position
            dx = self.target.x - telemetry_data.x
            dy = self.target.y - telemetry_data.y

            # Target angle (yaw needed to face the target)
            # Yaw is measured from x-axis, counter-clockwise positive
            target_yaw = math.atan2(dy, dx)

            # Calculate relative angle (difference between target yaw and current yaw)
            delta_yaw = target_yaw - telemetry_data.yaw

            # Normalize to [-pi, pi]
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            # Convert to degrees
            delta_yaw_deg = math.degrees(delta_yaw)

            if abs(delta_yaw_deg) > self.angle_tolerance:
                # Determine direction (1 = counter-clockwise, -1 = clockwise)
                direction = 1 if delta_yaw_deg >= 0 else -1

                # Send COMMAND_LONG with MAV_CMD_CONDITION_YAW
                self.connection.mav.command_long_send(
                    1,  # target_system
                    0,  # target_component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,  # confirmation
                    abs(delta_yaw_deg),  # param1: target angle in degrees
                    self.turning_speed,  # param2: angular speed in deg/s
                    direction,  # param3: direction (-1 = CW, 1 = CCW)
                    1,  # param4: 0 = absolute, 1 = relative
                    0,  # param5: empty
                    0,  # param6: empty
                    0,  # param7: empty
                )
                return f"CHANGE_YAW: {delta_yaw_deg}"

        return None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
