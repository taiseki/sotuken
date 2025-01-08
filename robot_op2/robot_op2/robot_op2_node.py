import rclpy
import serial
import math
import struct
from pynput import keyboard

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Char
from std_msgs.msg import String
from std_msgs.msg import Float32
from turtlesim.msg import Pose as NPose#X, Y, Theta tukau

###############################################
#ロボットの操縦方法
#  W E
#A S D
#  X
#基本的にWASDで操作
#W,A同時押しで左前進み等、同時押しで斜め対応
#X押しながらA or Dで回転
#Eは非常停止、即止まり
###############################################

#define したい
WEST        = 0b00000001
NORTH       = 0b00000010
SOUTH       = 0b00000100
EAST        = 0b00001000
NORTH_WEST  = 0b00000011
NORTH_EAST  = 0b00001010
SOUTH_WEST  = 0b00000101
SOUTH_EAST  = 0b00001100
ROTATE_L    = 0b00010001
ROTATE_R    = 0b00011000
EMERGENCY   = 0b10000000

class OdomOpNode(Node):
    def __init__(self):
        super().__init__('robot_op_node')
        self.pub = self.create_publisher(NPose, 'op', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.msg_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.presskey = 0
        self.pwm = '0'
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.rcv_flg = 0
        self.robot_pose = NPose()
        self.ser.read_all() #ゴミを最初に全部読み取る
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.abx = 0.0
        self.aby = 0.0

    def msg_callback(self, msg):
        print("[cmd_vel][debug]linear.x", msg.linear.x, "linear.y", msg.linear.y, "angular.z", msg.angular.z)
        
        if (msg.linear.x + msg.linear.y + msg.angular.z) == 0.0:
            self.ser.write('x'.encode())
            return
        ##########メカナムの運動学###########
        #|v0|   | 1 -1  lx + ly ||x.|
        #|v1| = | 1  1  lx + ly ||y.|
        #|v2|   |-1 -1  lx + ly ||T.|
        #|v3|   |-1  1  lx + ly |
        #lx = 0.047, ly = 0.1045 lx+ly = 0.1515
        #####################################

        v0 = int(100*(2 * msg.linear.x -  2 * msg.linear.y + msg.angular.z))
        v1 = int(100*(2 * msg.linear.x + 2 * msg.linear.y + msg.angular.z))
        v2 = int(100*(-2 * msg.linear.x - 2 * msg.linear.y + msg.angular.z))
        v3 = int(100*(-2 * msg.linear.x + 2 * msg.linear.y + msg.angular.z))
        #最大値 127
        v0 = 127 if v0 > 127 else v0
        v1 = 127 if v1 > 127 else v1
        v2 = 127 if v2 > 127 else v2
        v3 = 127 if v3 > 127 else v3
        #最小値 -127
        v0 = -127 if v0 < -127 else v0
        v1 = -127 if v1 < -127 else v1
        v2 = -127 if v2 < -127 else v2
        v3 = -127 if v3 < -127 else v3
        #1byteのマイナスの値に変換 
        v0 = -((v0 ^ 0b11111111) + 1) if v0 < 0 else v0 
        v1 = -((v1 ^ 0b11111111) + 1) if v1 < 0 else v1
        v2 = -((v2 ^ 0b11111111) + 1) if v2 < 0 else v2
        v3 = -((v3 ^ 0b11111111) + 1) if v3 < 0 else v3

        print("[cmd_vel][debug] v0 =", v0, bin(v0))
        print("[cmd_vel][debug] v1 =", v1, bin(v1))
        print("[cmd_vel][debug] v2 =", v2, bin(v2))
        print("[cmd_vel][debug] v3 =", v3, bin(v3))
        
        data = [0x6E, v0, v1, v2, v3]
        send_data = struct.pack('BBBBB', *data)
        #print(bin(data[1]))
        self.ser.write(data)
        
        # self.ser.write('n'.encode()) 
        # self.ser.write(int(127*(msg.linear.x - msg.linear.y + msg.angular.z)))
        # self.ser.write(int(127*(msg.linear.x + msg.linear.y + msg.angular.z)))
        # self.ser.write(int(127*(-msg.linear.x - msg.linear.y + msg.angular.z)))
        # self.ser.write(int(127*(-msg.linear.x + msg.linear.y + msg.angular.z)))

    def timer_callback(self):
        c = self.ser.readline()
        try:
            print(c.decode()) #debug
            print(float(c[2:9].decode()))
            if(c[0] == 120): #120:ascii 'x'
                self.x = float(c[2:9].decode())
                self.rcv_flg |= 0b001
            elif(c[0] == 121): #121:ascii 'y'
                self.y = float(c[2:9].decode())
                self.rcv_flg |= 0b010
            elif(c[0] == 84): #84 :ascii 'T'
                self.theta += float(c[2:9].decode())
                self.rcv_flg |= 0b100

            if(self.rcv_flg == 0b111):
                self.robot_pose.x += self.x * math.cos(self.theta) + self.y * math.cos(self.theta + 1.57079632679) #y軸はx軸と９０度 PI / 2 = 1.570796326794...
                self.robot_pose.y += self.x * math.sin(self.theta) + self.y * math.sin(self.theta + 1.57079632679)
                # self.robot_pose.x += math.sqrt(self.x*self.x + self.y*self.y)*math.cos(self.theta)
                # self.robot_pose.y += math.sqrt(self.x*self.x + self.y*self.y)*math.sin(self.theta)
                self.robot_pose.theta = self.theta
                self.pub.publish(self.robot_pose)
                self.rcv_flg = 0
        except ValueError:
            print("str to Float hennkann dekinakattayo")
            # data = [0x30, 0x30, 0x30, 0x6E, 0x30, 0x30, 0x30, 0x30] #'n', '0', '0', '0', '0'
            # self.ser.write(bytes(data))
        
        
        
    def on_press(self, key):
        try:
            print(f'key {key} is pressed')
            if key.char == 'a':
                self.presskey |= 0b00000001
            elif key.char == 'w':
                self.presskey |= 0b00000010
            elif key.char == 's':
                self.presskey |= 0b00000100
            elif key.char == 'd':
                self.presskey |= 0b00001000
            elif key.char == 'x':
                self.presskey |= 0b00010000
            elif key.char == 'f':
                self.presskey |= 0b00100000
            elif key.char == 'g':
                self.presskey |= 0b01000000
            elif key.char == 'e':
                self.presskey |= 0b10000000
            elif key.char == '0' or \
                 key.char == '1' or \
                 key.char == '2' or \
                 key.char == '3' or \
                 key.char == '4' or \
                 key.char == '5' or \
                 key.char == '6' or \
                 key.char == '7' or \
                 key.char == '8' or \
                 key.char == '9':
                self.pwm = key.char
        except AttributeError:
            print('special key {0} pressed'.format(key))



    def on_release(self, key):
        try:
            print(f'key {key} is released')
            if key.char == 'a':
                self.presskey &= 0b11111110
            elif key.char == 'w':
                self.presskey &= 0b11111101
            elif key.char == 's':
                self.presskey &= 0b11111011
            elif key.char == 'd':
                self.presskey &= 0b11110111
            elif key.char == 'x':
                self.presskey &= 0b11101111
            elif key.char == 'f':
                self.presskey &= 0b11011111
            elif key.char == 'g':
                self.presskey &= 0b10111111
            elif key.char == 'e':
                self.presskey &= 0b01111111
        except AttributeError:
            print('special key {0} released'.format(key))


    # with keyboard.Listener(on_press = on_press, on_release = on_release) as listener:
    #     listener.join()
    


    def KeyboardLoop(self):
        listener = keyboard.Listener(on_press = self.on_press, on_release = self.on_release)
        listener.start()
        oldstate = self.presskey
        oldpwm = self.pwm
        while rclpy.ok():
            if(self.presskey != oldstate or self.pwm != oldpwm):
                if(self.presskey & EMERGENCY):
                    print("EMERGENCY")
                    self.ser.write('x'.encode()) 
                    self.pwm = '0'
                elif(self.presskey == WEST):
                    print("WEST")
                    self.ser.write('a'.encode())
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == NORTH):
                    print("NORTH")
                    self.ser.write('w'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == SOUTH):
                    print("SOUTH")
                    self.ser.write('s'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == EAST):
                    print("EAST")
                    self.ser.write('d'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == NORTH_WEST):
                    print("NORTH_WEST")
                    self.ser.write('q'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == NORTH_EAST):
                    print("NORTH_EAST")
                    self.ser.write('e'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == SOUTH_WEST):
                    print("SOUTH_WEST")
                    self.ser.write('z'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == SOUTH_EAST):
                    print("SOUTH_EAST")
                    self.ser.write('c'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == ROTATE_L):
                    print("ROTATE_L")
                    self.ser.write('f'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == ROTATE_R):
                    print("ROTATE_R")
                    self.ser.write('g'.encode()) 
                    self.ser.write(self.pwm.encode()) 
                elif(self.presskey == 0 or self.presskey == 0b00010000): #なにも入力してないかｘだけ押してるとき
                    self.ser.write('x'.encode())
                    print("NUTRAL")
                else:
                    print("nannka nyuuryoku tigauyo")
                print(self.presskey)
                oldstate = self.presskey
                oldpwm = self.pwm
            rclpy.spin_once(self)

def main():
    rclpy.init()
    node = OdomOpNode()
    try:
        node.KeyboardLoop()
    except KeyboardInterrupt:
        print('Ctrl+C')
    node.destroy_node() #destroy
    rclpy.shutdown()

if __name__ == '__main__':
    main()