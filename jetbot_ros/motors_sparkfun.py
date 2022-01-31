import rclpy

from rclpy.node import Node
from jetbot_ros.motors import MotorController

import qwiic

class MotorControllerSparkFun(MotorController):
    """
    Motor controller node that supports the SparkFun Qwiic Motor Driver (SparkFun JetBot).
    @see motors.py for the base class to implement different controllers.
    """
    MOTOR_LEFT = 0      # left motor ID
    MOTOR_RIGHT = 1     # right motor ID
    MAX_SPEED = 255
    
    def __init__(self):
        super().__init__()

        self.declare_parameter('max_rpm', 140)              # https://www.sparkfun.com/products/13302
        self.declare_parameter('wheel_separation', 0.1016)  # 4 inches in meters
        self.declare_parameter('wheel_diameter', 0.06604)   # 2.6 inches in meters
        
        # Scan for devices on I2C bus
        addresses = qwiic.scan()

        if 93 in addresses:
            # get motor objects from driver
            self.motors = qwiic.QwiicScmd()
        else:
            raise KeyError('Qwiic Motor Controller not found on i2c bus')


    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-255, 255]
        """
        self.motors.set_drive(self.MOTOR_LEFT, 0, self._scale_speed(left))
        self.motors.set_drive(self.MOTOR_RIGHT, 0, self._scale_speed(right))
        
        self.motors.enable()

    def stop(self):
        """
        Sets the motor speeds to 0 and then stops 
        """
        self.motors.set_drive(self.MOTOR_LEFT,0,0)
        self.motors.set_drive(self.MOTOR_RIGHT,0,0)
        self.motors.disable()

    def _scale_speed(self, value):
        return int(value * self.MAX_SPEED)
 

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControllerSparkFun()
    node.get_logger().info("listening for velocity messages...")
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    
	

