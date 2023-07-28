
class BaseDrone(object):
    '''This is the base class of the drone class, which contains the most basic operations of the drone.

       这是无人机类的基类，包含无人机的共同属性和方法'''
    
    def __init__(self, name, max_speed=100):
        self.name = name
        self.max_speed = max_speed
        self.speed_ranger = lambda a: max_speed if a > max_speed else (-max_speed if -a > max_speed else a)

    def take_off(self):
        print(f"{self.name} is taking off.")
        # Do sth. to taking off

    def land(self):
        print(f"{self.name} is landing.")
        # Do sth. to landing
    
    def arm(self):
        print(f"{self.name} is arming.")
        # Do sth. to arm

    def set_max_speed(self, max_speed):
        self.max_speed = max_speed
        self.ranger = lambda a: max_speed if a > max_speed else (-max_speed if -a > max_speed else a)

    def set_name(self, name):
        self.name = name