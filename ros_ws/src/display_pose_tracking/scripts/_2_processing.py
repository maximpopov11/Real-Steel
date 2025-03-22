from typing import Dict
from model import Bodypoints_t, Robot_Angles_t
from custom_types import ts_Bodypoints_t, q_Robot_Angles_t
from threading import Thread
from asyncio import sleep
from heapq import heappush

async def generate_bodypoints(bodypoints_by_timestamp: ts_Bodypoints_t, robot_angles_queue: q_Robot_Angles_t,):
    await sleep(.02)

    heappush(robot_angles_queue, (bodypoints_by_timestamp[0], [0, 0, 0, 0]))

def process_bodypoints(
    bodypoints_by_timestamp: ts_Bodypoints_t,
    robot_angles_queue: q_Robot_Angles_t,
):
    """
    Spawn threads to process Bodypoints and load them into the RobotAngles queue.
    """
    angle_thread = Thread(target=generate_bodypoints, args=(bodypoints_by_timestamp, robot_angles_queue))
    angle_thread.run()
    pass


def find_missing_points(
    bodypoints_by_timestamp: Dict[int, Bodypoints_t], timestamp: int
) -> Bodypoints_t:
    """
    Use points in surrounding frames to populate guesses for any unfound bodypoints.
    """
    pass


def smooth_points(bodypoint_queue: Dict[int, Bodypoints_t], index: int) -> Bodypoints_t:
    """
    Use points in surrounding frames to smoothen points, ignoring differences within a small margin.
    """
    pass


def get_robotangles(bodypoints: Bodypoints_t) -> Robot_Angles_t:
    """
    Map the given bodypoints to robot angles.
    """
    pass


def restrain_angles(robot_angles: Robot_Angles_t) -> Robot_Angles_t:
    """
    Restrain the given robot_angles to disallow any infeasible angles.
    """
    pass


def restrain_position(bodypoints: Bodypoints_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow contact with the self or with the cord at the back of the head.
    """
    pass


def restrain_speed(former_bodypoints: Bodypoints_t, bodypoints: Bodypoints_t) -> Bodypoints_t:
    """
    Restrain the given bodypoints to disallow moving faster than the threshold.
    """
    pass
