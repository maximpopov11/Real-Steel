from time import time

SPEED_THRESHOLD = 2.0  # rad/s

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)

def interpolate_points(start_angles, end_angles, num_steps):
    if num_steps <= 0:
        return []
    interpolated = []
    for step in range(1, num_steps + 1):
        alpha = step / num_steps  # Ranges from 1/num_steps to 1.0
        interp = [s + alpha * (e - s) for s, e in zip(start_angles, end_angles)]
        interpolated.append(interp)
    return interpolated