import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
class Motor_cmds(Node):
    def __init__(self):
        super().__init__("motor_pwms")
        self.teleop_cmds = self.create_subscription(Twist, "/cmd_vel", self.convert_to_pwm, qos_profile=1)
        self.motion_cmds = self.create_subscription(Float32MultiArray, "/vr_vl", self.convert_to_pwm, qos_profile=1)
        self.motor_pwm_vals = self.create_publisher(Float32MultiArray, "/motor_pwm_vals", qos_profile=1)
        # self.timer = self.create_timer(0.5, self.setPwm)

    # def setPwm(self):
    #     # self.pwmR = pwmData[0]
    #     # self.pwmL = pwmData[1]
    #     msg = String()
    #     msg.data = "100-125"
    #     self.motor_pwm_vals.publish(msg)

    def convert_to_pwm(self, cmd):
        pwm_max = 255
        pwm_min = 180
        cmd_max = 1
        cmd_min = 0
        if type(cmd) is Twist:
            #add diff_drive equations to get vr/vl in pwm form
            #publish the pwm values on the motor_pwm_vals topic
            v = cmd.linear.x
            pubV = Float32MultiArray()
            pwm_out = v * (pwm_max - pwm_min) + pwm_min
            pubV.data = [pwm_out]
            self.motor_pwm_vals.publish(pubV)

        elif type(cmd) is Float32MultiArray:
            #publish pwm values on the motor_pwm_vals topic
            pass

def main(args=None):
    rclpy.init(args=args)
    motor_pwms = Motor_cmds()
    rclpy.spin(motor_pwms)
    motor_pwms.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
