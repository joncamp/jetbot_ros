import rclpy
import qwiic_micro_oled

from jetbot_ros.oled import OLEDController

import subprocess


class OLEDControllerSparkFun(OLEDController):
    """
    OLED controller node for SparkFun Micro OLED Breakout (used on the SparkFun JetBot)
    @see oled.py for the base class to impelement different OLED controllers.
    """
    user_text = None

    def __init__(self):
        super().__init__()
        
        # 68x64 display with hardware I2C:
        self.display = qwiic_micro_oled.QwiicMicroOled()
        
        self.display.begin()
        self.display.clear(self.display.PAGE)
        self.display.clear(self.display.ALL)

        # Set Font
        self.display.set_font_type(0)
        # Could replace line spacing with disp2.getFontHeight, but doesn't scale properly

        self.display.display()
        
    def render(self, text):
        # Draw a black filled box to clear the image.
        self.display.clear(self.display.PAGE)

        # Draw text
        self.display.print(text)

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
    