from typing import Dict
from custom_types import ts_Bodypoints_t, q_Robot_Angles_t, Bodypoints_t, Robot_Angles_t, q_Bodypoints_t
from asyncio import sleep
from heapq import heappush

async def generate_mock_bodypoints(bodypoints_by_timestamp: ts_Bodypoints_t, robot_angles_queue: q_Robot_Angles_t,):
    await sleep(.02)

    heappush(robot_angles_queue, (bodypoints_by_timestamp[0], [0, 0, 0, 0]))

# TODO: Our bodypoints_by_timestamp is a heap, but I want a dict for efficient access to arbitrary timestamps, can we just dict it instead?
# TODO: we need to add locks to all of our multi-threaded heaps
def process_bodypoints(
    bodypoints_by_timestamp: ts_Bodypoints_t,
    robot_angles_queue: q_Robot_Angles_t,
):
    """
    Spawn threads to process Bodypoints and load them into the RobotAngles queue.
    """
    robot_bodypoints_by_timestamp: dict[int, ts_Bodypoints_t] = {}

    # TODO: thread max_parallelism at a time, wait if all done until more come in
    # TODO: are we ever looking to the future for things we might have already calculated? If so, do they ever wait and look at the past? How much complexity right now?
    timestamp = next(iter(bodypoints_by_timestamp.keys()))
    
    find_missing_points(bodypoints_by_timestamp, timestamp)
    smooth_points(bodypoints_by_timestamp, timestamp)
    points = bodypoints_by_timestamp[timestamp]

    robot_angles = get_robotangles(points)
    restrained_angles = restrain_angles(robot_angles)
    robot_bodypoints = restrain_position(restrained_angles)
    robot_bodypoints_by_timestamp[timestamp] = robot_bodypoints

    final_bodypoints = restrain_speed(robot_bodypoints_by_timestamp, timestamp)
    robot_bodypoints_by_timestamp[timestamp] = final_bodypoints

    final_angles = get_robotangles_from_robot_bodypoints(final_bodypoints)
    heappush(robot_angles_queue, (timestamp, final_angles))
    
    del bodypoints_by_timestamp[timestamp]


# TODO: these aren't all working in a consistent manner, let's be consistent
def find_missing_points(bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints at the timestamp.
    """
    pass


def smooth_points(bodypoint_queue: Dict[int, Bodypoints_t], timestamp: int):
    """
    Use points in surrounding frames to smoothen points at the timestamp, ignoring differences within a small margin.
    """
    pass


def get_robotangles(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def get_robotangles_from_robot_bodypoints(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given robot bodypoints to robot angles.
    """
    pass


def restrain_angles(robot_angles: Robot_Angles_t) -> Robot_Angles_t:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def restrain_position(robot_angles: Robot_Angles_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    This will use the given robot_angles to calculate the robot's bodypoints and restrain them as necessary.
    """
    # TODO: calculate robot bodypoints (or type, we don't need all 33 points), then restrain them, and use them in restrain_speed too
    pass


def restrain_speed(robot_bodypoints_by_timestamp: dict[int, ts_Bodypoints_t], timestamp: int) -> Bodypoints_t:
    """
    Restrain the robot bodypoints at the given timestamp to disallow moving faster than the threshold.
    """
    pass
