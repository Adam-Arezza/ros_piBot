import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorPwm(Node):
    def __init__(self):
        super().__init__("motor_pwm")
        self.subscription = self.create_subscription(String, "pwm", self.setPwm, 10)
        # self.motor_pwm_vals = self.create_publisher(String, "pwm_cmds", qos_profile=None, )

    def setPwm(self, pwmData):
        self.pwmR = pwmData[0]
        self.pwmL = pwmData[1]

def main(args=None):
    rclpy.init(args=args)
    motor_pwms = MotorPwm()
    rclpy.spin(motor_pwms)
    motor_pwms.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
