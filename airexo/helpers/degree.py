"""
Angle Degree Transformation Functions.

Authors: Hongjie Fang.
"""

import math


def deg_2_rad(x):
    """
    Transform the degree into rad.
    """
    return x / 180 * math.pi

def rad_2_deg(x):
    """
    Transform the rad into degree.
    """
    return x / math.pi * 180

def deg_clip(x, w0 = True):
    """
    Clip the degree into range [0, 360) or (0, 360] (specified by parameter w0: whether include 0 in the range).
    """
    x = x - 360 * math.ceil((x - 360 + 1e-8) / 360)
    if x == 0 and not w0:
        x = 360
    return x

def deg_distance(x, y, direction, w0 = True):
    """
    Get the degree distance from degree x to degree y, according to the given direction. 
    Here we assume that the distance is in range [0, 360) or (0, 360] 
    (specified by parameter w0: whether include 0 in the range).
    """
    dis = (deg_clip(y) - deg_clip(x)) * direction
    if dis < 0 or (dis == 0 and not w0):
        dis = dis + 360
    return dis

def deg_check_range(x, xmin, xmax, direction):
    """
    Check whether the angle degree x is in the given range [xmin, xmax], according to the given direction. 
    Here we assume that the range is not larger than 360 degrees.

    [Examples]
    - for x = 50, [xmin, xmax] = [10, 100], direction = 1, return: True.
    - for x = 150, [xmin, xmax] = [10, 100], direction = 1, return: False
    - for x = 50, [xmin, xmax] = [250, 100], direction = 1, return: True.
    - for x = 150, [xmin, xmax] = [250, 100], direction = 1, return: False.
    - for x = 50, [xmin, xmax] = [10, 100], direction = -1, return: False.
    - for x = 150, [xmin, xmax] = [10, 100], direction = -1, return: True.
    - for x = 50, [xmin, xmax] = [250, 100], direction = -1, return: False.
    - for x = 150, [xmin, xmax] = [250, 100], direction = -1, return: True.
    """
    if direction == -1:
        xmin, xmax = xmax, xmin
    return (x >= xmin and x <= xmax) or (0 <= x and x <= xmax and xmax < xmin) or (xmax < xmin and xmin <= x and x <= 360)

def deg_clip_in_range(x, xmin, xmax, direction):
    """
    Clip the angle degree x into the given range [xmin, xmax], according to the given direction. 
    Here we assume that the range is not larger than 360 degrees.
    
    [Examples]
    - for x = 50, [xmin, xmax] = [10, 100], direction = 1, return: 50.
    - for x = 150, [xmin, xmax] = [10, 100], direction = 1, return: 100.
    - for x = 50, [xmin, xmax] = [250, 100], direction = 1, return: 50.
    - for x = 150, [xmin, xmax] = [250, 100], direction = 1, return: 100.
    - for x = 50, [xmin, xmax] = [10, 100], direction = -1, return: 10.
    - for x = 150, [xmin, xmax] = [10, 100], direction = -1, return: 150.
    - for x = 50, [xmin, xmax] = [250, 100], direction = -1, return: 100.
    - for x = 150, [xmin, xmax] = [250, 100], direction = -1, return: 150.
    """
    x = deg_clip(x)
    if deg_check_range(x, xmin, xmax, direction):
        return x
    dxmin = deg_distance(x, xmin, direction)
    dxmax = deg_distance(xmax, x, direction)
    return xmin if dxmin <= dxmax else xmax

def deg_percentile(x, xmin, xmax, direction):
    """
    Calculate the degree percentile of x given the expected range [xmin, xmax], according to the given direction. Notice the return is not necessarily in range [0, 1] because this function does not consider out-of-range situations. Refer to "deg_check_range" for more details.
    Here we assume that the range is not larger than 360 degrees.
    
    [Examples]
    - for x = 50, [xmin, xmax] = [10, 100], direction = 1, the percentile is 4/9.
    - for x = 50, [xmin, xmax] = [250, 100], direction = 1, the percentile is 16/21.
    """
    return deg_distance(xmin, x, direction, w0 = True) / deg_distance(xmin, xmax, direction, w0 = False)

def deg_zero_centered(x, xmin, xmax, xdir):
    """
    Transform the degree in [xmin, xmax] into zero-centered equivalent degree.
    Here we assume that the range is not larger than 360 degrees.
    """
    assert deg_check_range(x, xmin, xmax, xdir) and deg_check_range(0, xmin, xmax, xdir)
    neg_bound = max(xmin, xmax)
    return x - 360 if x >= neg_bound else x
