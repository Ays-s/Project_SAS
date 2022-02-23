def nearest_angle(angle):
    """ return the nerest right angle from angle"""
    while angle > 180: angle -= 360
    while angle <-180: angle += 360
    if -45 < angle <= 45:
        return 0
    elif 45 < angle <= 135:
        return 90
    elif -135 < angle <= -45:
        return -90
    else:
        return 180

