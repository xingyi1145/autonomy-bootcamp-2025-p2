"""
Bootcamp F2025

Main process to setup and manage all the other working processes
"""

import multiprocessing as mp
import queue
import time

from pymavlink import mavutil

from modules.common.modules.logger import logger
from modules.common.modules.logger import logger_main_setup
from modules.common.modules.read_yaml import read_yaml
from modules.command import command
from modules.command import command_worker
from modules.heartbeat import heartbeat_receiver_worker
from modules.heartbeat import heartbeat_sender_worker
from modules.telemetry import telemetry_worker
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from utilities.workers import worker_manager


# MAVLink connection
CONNECTION_STRING = "tcp:localhost:12345"

# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
# Set queue max sizes (<= 0 for infinity)
HEARTBEAT_RECEIVER_TO_MAIN_QUEUE_MAX_SIZE = 10
TELEMETRY_TO_COMMAND_QUEUE_MAX_SIZE = 10
COMMAND_TO_MAIN_QUEUE_MAX_SIZE = 10

# Set worker counts
HEARTBEAT_SENDER_WORKER_COUNT = 1
HEARTBEAT_RECEIVER_WORKER_COUNT = 1
TELEMETRY_WORKER_COUNT = 1
COMMAND_WORKER_COUNT = 1

# Any other constants
TARGET = command.Position(10, 20, 30)
RUN_TIME = 100  # seconds

# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================


def main() -> int:
    """
    Main function.
    """
    # Configuration settings
    result, config = read_yaml.open_config(logger.CONFIG_FILE_PATH)
    if not result:
        print("ERROR: Failed to load configuration file")
        return -1

    # Get Pylance to stop complaining
    assert config is not None

    # Setup main logger
    result, main_logger, _ = logger_main_setup.setup_main_logger(config)
    if not result:
        print("ERROR: Failed to create main logger")
        return -1

    # Get Pylance to stop complaining
    assert main_logger is not None

    # Create a connection to the drone. Assume that this is safe to pass around to all processes
    # In reality, this will not work, but to simplify the bootamp, preetend it is allowed
    # To test, you will run each of your workers individually to see if they work
    # (test "drones" are provided for you test your workers)
    # NOTE: If you want to have type annotations for the connection, it is of type mavutil.mavfile
    connection = mavutil.mavlink_connection(CONNECTION_STRING)
    connection.wait_heartbeat(timeout=30)  # Wait for the "drone" to connect

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Create a worker controller
    controller = worker_controller.WorkerController()

    # Create a multiprocess manager for synchronized queues
    mp_manager = mp.Manager()

    # Create queues
    heartbeat_receiver_to_main_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        HEARTBEAT_RECEIVER_TO_MAIN_QUEUE_MAX_SIZE,
    )
    telemetry_to_command_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        TELEMETRY_TO_COMMAND_QUEUE_MAX_SIZE,
    )
    command_to_main_queue = queue_proxy_wrapper.QueueProxyWrapper(
        mp_manager,
        COMMAND_TO_MAIN_QUEUE_MAX_SIZE,
    )

    # Create worker properties for each worker type (what inputs it takes, how many workers)
    # Heartbeat sender
    result, heartbeat_sender_worker_properties = worker_manager.WorkerProperties.create(
        count=HEARTBEAT_SENDER_WORKER_COUNT,
        target=heartbeat_sender_worker.heartbeat_sender_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Heartbeat Sender")
        return -1

    assert heartbeat_sender_worker_properties is not None

    # Heartbeat receiver
    result, heartbeat_receiver_worker_properties = worker_manager.WorkerProperties.create(
        count=HEARTBEAT_RECEIVER_WORKER_COUNT,
        target=heartbeat_receiver_worker.heartbeat_receiver_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[heartbeat_receiver_to_main_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Heartbeat Receiver")
        return -1

    assert heartbeat_receiver_worker_properties is not None

    # Telemetry
    result, telemetry_worker_properties = worker_manager.WorkerProperties.create(
        count=TELEMETRY_WORKER_COUNT,
        target=telemetry_worker.telemetry_worker,
        work_arguments=(connection,),
        input_queues=[],
        output_queues=[telemetry_to_command_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Telemetry")
        return -1

    assert telemetry_worker_properties is not None

    # Command
    result, command_worker_properties = worker_manager.WorkerProperties.create(
        count=COMMAND_WORKER_COUNT,
        target=command_worker.command_worker,
        work_arguments=(connection, TARGET),
        input_queues=[telemetry_to_command_queue],
        output_queues=[command_to_main_queue],
        controller=controller,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create arguments for Command")
        return -1

    assert command_worker_properties is not None

    # Create the workers (processes) and obtain their managers
    worker_managers: list[worker_manager.WorkerManager] = []

    result, heartbeat_sender_manager = worker_manager.WorkerManager.create(
        worker_properties=heartbeat_sender_worker_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Heartbeat Sender")
        return -1

    assert heartbeat_sender_manager is not None
    worker_managers.append(heartbeat_sender_manager)

    result, heartbeat_receiver_manager = worker_manager.WorkerManager.create(
        worker_properties=heartbeat_receiver_worker_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Heartbeat Receiver")
        return -1

    assert heartbeat_receiver_manager is not None
    worker_managers.append(heartbeat_receiver_manager)

    result, telemetry_manager = worker_manager.WorkerManager.create(
        worker_properties=telemetry_worker_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Telemetry")
        return -1

    assert telemetry_manager is not None
    worker_managers.append(telemetry_manager)

    result, command_manager = worker_manager.WorkerManager.create(
        worker_properties=command_worker_properties,
        local_logger=main_logger,
    )
    if not result:
        print("Failed to create manager for Command")
        return -1

    assert command_manager is not None
    worker_managers.append(command_manager)

    # Start worker processes
    for manager in worker_managers:
        manager.start_workers()

    main_logger.info("Started")

    # Main's work: read from all queues that output to main, and log any commands that we make
    # Continue running for 100 seconds or until the drone disconnects
    start_time = time.time()
    was_connected = False  # Track if we were ever connected
    should_exit = False

    while time.time() - start_time < RUN_TIME and not should_exit:
        # Read heartbeat receiver status
        try:
            status = heartbeat_receiver_to_main_queue.queue.get(timeout=0.1)
            if status is not None:
                main_logger.info(f"Connection status: {status}")
                if status == "Connected":
                    was_connected = True
                elif status == "Disconnected" and was_connected:
                    # Only exit if we were connected before and then disconnected
                    should_exit = True
        except queue.Empty:
            pass

        # Read command outputs
        try:
            command_result = command_to_main_queue.queue.get(timeout=0.1)
            if command_result is not None:
                main_logger.info(f"Command: {command_result}")
        except queue.Empty:
            pass

    # Stop the processes
    controller.request_exit()

    main_logger.info("Requested exit")

    # Fill and drain queues from END TO START
    command_to_main_queue.fill_and_drain_queue()
    telemetry_to_command_queue.fill_and_drain_queue()
    heartbeat_receiver_to_main_queue.fill_and_drain_queue()

    main_logger.info("Queues cleared")

    # Clean up worker processes
    for manager in worker_managers:
        manager.join_workers()

    main_logger.info("Stopped")

    # We can reset controller in case we want to reuse it
    # Alternatively, create a new WorkerController instance
    controller.clear_exit()

    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Failed with return code {result_main}")
    else:
        print("Success!")
