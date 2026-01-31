"""
Command worker to make decisions based on Telemetry Data.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import command
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def command_worker(
    connection: mavutil.mavfile,
    target: command.Position,
    input_queue: queue_proxy_wrapper.QueueProxyWrapper,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    connection: MAVLink connection to the drone.
    target: Target position to face.
    input_queue: Queue to receive TelemetryData from telemetry worker.
    output_queue: Queue to output command strings to main process.
    controller: Worker controller for pause/exit signals.
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (command.Command)

    result, cmd = command.Command.create(connection, target, local_logger)
    if not result or cmd is None:
        local_logger.error("Failed to create Command")
        return

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()

        # Get telemetry data from input queue
        try:
            telemetry_data = input_queue.queue.get(timeout=0.1)
            if telemetry_data is None:
                continue

            local_logger.info(f"Received TelemetryData: {telemetry_data}")

            # Process the telemetry data
            result_str = cmd.run(telemetry_data)

            if result_str is not None:
                local_logger.info(f"Command result: {result_str}")
                output_queue.queue.put(result_str)
        except:  # pylint: disable=bare-except
            # Queue timeout or empty, continue
            pass

    local_logger.info("Worker exiting")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
