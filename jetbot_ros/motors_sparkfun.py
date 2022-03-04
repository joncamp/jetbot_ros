import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from jetbot_ros.motors import MotorController

import qwiic_scmd

class MotorControllerSparkFun(MotorController):
    """
    Motor controller node that supports the SparkFun Qwiic Motor Driver (SparkFun JetBot).
    @see motors.py for the base class to implement different controllers.
    """
    MOTOR_LEFT = 0      # left motor ID
    MOTOR_RIGHT = 1     # right motor ID
    MAX_SPEED = 255
    
    def __init__(self, parameter_overrides=[
            Parameter('max_rpm', value=140),                # https://www.sparkfun.com/products/13302
            Parameter('wheel_separation', value=0.1016),    # 4 inches in meters    
            Parameter('wheel_diameter', value=0.06604)      # 2.6 inches in meters 
        ]):
        super().__init__()
        
        self.motors = qwiic_scmd.QwiicScmd()

        self.get_logger().info("SCMD Motors Initialized!")

    def set_speed(self, left, right):
        """
        Sets the motor speeds between [-255, 255]
        """
        self.get_logger().info(f"set_speed({left}, {right})")

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
    
	

