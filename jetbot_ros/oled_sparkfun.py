import rclpy
import qwiic_micro_oled

from jetbot_ros.oled import OLEDController

import subprocess


class OLEDControllerSparkFun(OLEDController):
    """
    OLED controller node for SparkFun Micro OLED Breakout (used on the SparkFun JetBot)
    @see oled.py for the base class to impelement different OLED controllers.
    """
    def __init__(self):
        super().__init__()
        
        # 68x64 display with hardware I2C:
        self.display = qwiic_micro_oled.QwiicMicroOled()

        if not self.display.connected:
            raise NotImplementedError('SparkFun controller failed to initialize!')
        else:
            self.display.begin()
            self.display.clear(self.display.ALL)
            self.display.clear(self.display.PAGE)

            # Set Font
            self.display.set_font_type(0)

            self.display.print("OLED Initialized!")
            self.get_logger().info("OLED Initialized!")

            self.display.display()
        
    def render(self, text):
        # Draw a black filled box to clear the image.
        self.display.clear(self.display.ALL)
        self.display.clear(self.display.PAGE)
        self.display.set_cursor(0, 0)

        # Draw text
        for idx, txt in enumerate(text):
            self.display.print(txt)

        # Present the image
        self.display.display()


def main(args=None):
    rclpy.init(args=args)
    
    node = OLEDControllerSparkFun()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
    