import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pigpio
import time
import serial
import threading


class Motor(Node):
    __uart_state = 0

    def __init__(self):
        super().__init__('motor_control')
        self.sub_vel = self.create_subscription(Twist,'cmd_vel',self.vel_callback,10)
        self.robot_wheel_pub = self.create_publisher(String,'robot_wheel_vel',10)
        self.wheel_width = 0.24       #width of robot
        self.left_vel = 0
        self.right_vel = 0
        self.real_left_vel = 0
        self.real_right_vel = 0
        self.dir = 0    #0, 1 , 2 ,3 ,4 ,5 ,6 ,7 =>  fw, bw, L, R, LF, RF, LB, RB

        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(19,100);
        self.pi.set_PWM_frequency(23,100);

        self.pi.set_mode(24, pigpio.OUTPUT)
        self.pi.set_mode(26, pigpio.OUTPUT)


        self.motor_val = [0,0]
        self.acc_param = 3

        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.ser.reset_input_buffer()


    def create_receive_threading(self):
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                self.__uart_state = 1
        except:
            print('---create_receive_threading error!---')
            pass



    def __receive_data(self):
        while True:
            line = self.ser.readline().decode('utf-8').rstrip()
            vals = line.split(",")
            if len(vals) != 4:
               return


            if self.dir == 0 or self.dir == 4 or self.dir == 5:           #**** edit from error in the book
               self.real_left_vel = float(vals[0])
               self.real_right_vel = float(vals[1])
            elif self.dir == 1 or self.dir == 6 or self.dir == 7:
               self.real_left_vel = -1 * float(vals[2])
               self.real_right_vel = -1 * float(vals[3])
            elif self.dir == 2:
               self.real_left_vel = -1 * float(vals[2])
               self.real_right_vel = float(vals[1])
            elif self.dir == 3:
               self.real_left_vel = float(vals[0])
               self.real_right_vel = -1 * float(vals[3])



            diff_left = abs(self.real_left_vel) - abs(self.left_vel)
            diff_right = abs(self.real_right_vel) - abs(self.right_vel)

            self.motor_val[0] = self.motor_val[0] - (diff_left * self.acc_param)
            if self.motor_val[0] < 0:
               self.motor_val[0] = 0

            if self.left_vel == 0:
               self.motor_val[0] = 0

            self.motor_val[1] = self.motor_val[1] - (diff_right * self.acc_param)
            if self.motor_val[1] < 0:
               self.motor_val[1] = 0

            if self.right_vel == 0:
               self.motor_val[1] = 0

            if self.left_vel > 0:
               self.pi.write(24,1)
            else:
               self.pi.write(24,0)


            if self.right_vel > 0:
               self.pi.write(26,0)
            else:
               self.pi.write(26,1)

            val_l = min(self.motor_val[0],255)
            val_r = min(self.motor_val[1],255)

            self.pi.set_PWM_dutycycle(23,val_l)
            self.pi.set_PWM_dutycycle(19,val_r)

            msg = String()

            msg.data = '%f,%f' % (self.real_left_vel, self.real_right_vel)

            self.robot_wheel_pub.publish(msg)



    def vel_callback(self, msg):
        vel_x = msg.linear.x
        vel_th = msg.angular.z

        if vel_x == 0:
           self.right_vel = vel_th * self.wheel_width / 2.0
           self.left_vel = (-1) * self.right_vel
        elif vel_th == 0:
           self.left_vel = self.right_vel = vel_x
        else:
           self.left_vel = vel_x - vel_th * self.wheel_width / 2.0
           self.right_vel = vel_x + vel_th * self.wheel_width / 2.0


        if self.left_vel > 0 and self.right_vel > 0:
           if self.left_vel == self.right_vel:
              self.dir = 0
           elif self.left_vel > self.right_vel:
              self.dir = 5
           else:
              self.dir = 4
        elif self.left_vel < 0 and self.right_vel < 0:
           if self.left_vel == self.right_vel:
              self.dir = 1
           elif self.left_vel > self.right_vel:
              self.dir = 7
           else:
              self.dir = 6
        elif self.left_vel > 0 and self.right_vel < 0:
           self.dir = 3
        elif self.left_vel < 0 and self.right_vel > 0:
           self.dir = 2


def main(args=None):
    rclpy.init(args=args)

    motor = Motor()
    motor.create_receive_threading()

    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
