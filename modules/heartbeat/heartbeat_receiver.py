"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to receive heartbeats and track connection status
    """

    __private_key = object()
    DISCONNECT_THRESHOLD = 5  # Number of missed heartbeats before disconnected

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        return True, cls(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self.connection = connection
        self.logger = local_logger
        self.missed_heartbeats = 0
        self.is_connected = False

    def run(
        self,
    ) -> str:
        """
        Attempt to receive a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        Returns "Connected" or "Disconnected".
        """
        # Try to receive a heartbeat with 1 second timeout
        msg = self.connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is not None:
            # Received a heartbeat
            self.missed_heartbeats = 0
            self.is_connected = True
            self.logger.info("Received heartbeat")
        else:
            # Missed a heartbeat
            self.missed_heartbeats += 1
            self.logger.warning(f"Missed heartbeat ({self.missed_heartbeats})")

            if self.missed_heartbeats >= self.DISCONNECT_THRESHOLD:
                self.is_connected = False

        status = "Connected" if self.is_connected else "Disconnected"
        return status


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
