class Jog:
    def __init__(self, config):
        self._speed = 0.
        self._acceleration = 0.
        self._distance_vector = [0., 0., 0.]
    # implement default acceleration / speed taken from stepper

    def set_speed(self, speed):
        self._speed = speed
    def set_acceleration(self, acceleration):
        self._acceleration = acceleration
    def set_distance_vector(self, dist):
        self._distance_vector = dist

    def reset(self):
        self._speed = 0.
        self._acceleration = 0.
        self._distance_vector = [0., 0., 0.]

    def get_speed(self):
        return self._speed
    def get_acceleration(self):
        return self._acceleration
    def get_distance_vector(self):
        return self._distance_vector

def load_config(config):
    return Jog(config)

