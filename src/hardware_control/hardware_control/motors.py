from smbus2 import SMBus
import rclpy
from rclpy.node import Node
from std_msgs.msg import _byte_multi_array

class MotorPwm(Node):
    def __init__(self):
        super().__init__("motor_pwm")
        self.bus = SMBus(1)
        self.subscription = self.create_subscription(_byte_multi_array, "pwm", self.setPwm, 10)

    def setPwm(self, pwmData):
        self.pwmR = pwmData[0]
        self.pwmL = pwmData[1]
        return self.sendCommand(self.pwmR, self.pwmL)

    def sendCommand(self, pwmR, pwmL):
        self.bus.write_i2c_block_data(0x08, 0, [self.pwmR, self.pwmL])

def main(args=None):
    rclpy.init(args=args)
    motor_pwms = MotorPwm()
    rclpy.spin(motor_pwms)
    motor_pwms.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()