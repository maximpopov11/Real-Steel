from time import time

def timestamp() -> int:
    """Returns the current time in milliseconds since the epoch using time()."""
    return int(time() * 1000)
