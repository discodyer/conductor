import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Vehicle, mavutil
from .BaseDrone import BaseDrone

class ArduCopter(BaseDrone):
    def __init__(self, connect_str, name, baudrate=115200, max_speed=100):
        super(ArduCopter, self).__init__(name, max_speed)
        while True:
            try:
                # 尝试连接飞控
                self.v = connect(connect_str, baud=baudrate, wait_ready=True)
                # 如果打开成功，就返回Vehicle对象，并且退出循环
                if isinstance(self.v, Vehicle):
                    print("{} 连接成功".format(str(name)))
                    break
            except Exception as e:
                # 如果打开失败，就打印错误信息
                print(e)
                print("Waiting for {} seconds before retrying...".format(3))
                time.sleep(3)
        
    def take_off(self, TargetAltitude):
        "起飞到指定高度,单位CM"
        # 发送起飞指令
        print(f"{self.name} is taking off to {TargetAltitude}CM")
        # simple_takeoff将发送指令，使无人机起飞并上升到目标高度
        self.v.simple_takeoff(TargetAltitude / 100)

    def land(self):
        "降落"
        print(f"{self.name} is landing.")
        self.v.mode = VehicleMode("LAND")

    def set_mode(self, mode):
        "设置飞行模式,一般用 'GUIDED' 模式操作"
        self.v.mode = VehicleMode(mode)

    def arm(self, mode='GUIDED'):
        "解锁电机并进入怠速"
        # 进行起飞前检查
        print("Basic pre-arm checks")
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        
        # 解锁无人机（电机将开始旋转）
        print("Arming motors")
        # 将无人机的飞行模式切换成"GUIDED"（一般建议在GUIDED模式下控制无人机）
        self.set_mode(mode)
        # 通过设置vehicle.armed状态变量为True，解锁无人机
        self.v.armed = True

        # 在无人机起飞之前，确认电机已经解锁
        while not self.v.armed:
            print(" Waiting for arming...")
            time.sleep(1)

    @property
    def get_Altitude(self):
        "获取飞行高度(测距仪)(CM)"
        # return self.v.location.global_relative_frame.alt
        return self.v.rangefinder.distance * 100

    @property
    def is_armable(self):
        """会检查飞控是否启动完成、有无GPS fix、卡曼滤波器
        若以上检查通过,则会返回True"""
        return self.v.is_armable
    
    def arm_and_takeoff(self, TargetAltitude, mode='GUIDED'):
        '''
        使无人机解锁并起飞到目标高度

        参数TargetAltitude即为目标高度,单位为CM,飞行模式默认为 GUIDED 
        
        并在无人机上升到目标高度之前，阻塞程序'''
        # 解锁无人机
        self.arm(mode)
        
        # 起飞无人机到指定高度（CM）
        self.take_off(TargetAltitude)

        # 在无人机上升到目标高度之前，阻塞程序
        while True:
            print(" Altitude: ", self.get_Altitude / 100, " M")
            # 当高度上升到目标高度的0.95倍时，即认为达到了目标高度，退出循环
            # vehicle.location.global_relative_frame.alt为相对于home点的高度
            if self.get_Altitude >= TargetAltitude * 0.95:
                print("Reached target altitude")
                break
            # 等待1s
            time.sleep(1)

    def velocity_control(self):
        pass

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.v.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle
        self.v.send_mavlink(msg)

    def position_control(self, velocity_x, velocity_y, velocity_z, duration):
        
        self.send_ned_velocity(velocity_x, velocity_y, velocity_z)

    def close(self):
        # 退出之前，清除vehicle对象
        self.v.close()
        pass