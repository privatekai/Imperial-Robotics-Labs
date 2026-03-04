import math

def dot_prod(a_x, a_y, b_x, b_y):
    return a_x * b_x + a_y * b_y

def magnitude(x, y):
    return math.sqrt(x**2 + y**2)

def projection(a_x, a_y, b_x, b_y):
    """
    Project vector b onto vector a
    """
    dot = dot_prod(a_x, a_y, b_x, b_y)
    a_mag = magnitude(a_x, a_y)
    
    proj_fac = dot / a_mag**2

    return proj_fac * a_x, proj_fac * a_y