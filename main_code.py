import rclpy
import sys
import select
import os
import time
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.0
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

settings = None

oper_key = 0
gap = 0
SCV = 0
TCV = 0

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

msg = """
              Height
        ┌────────────────┐
        │                │
        │                │
        │                │  
        │                │ Width
        │              ◇ │  
        │              ↑ │
        │              ◇ │
        └────────────────┘
"""
# console keyboard input function
def get_key(settings): # 거의 표준화된 키 입력 함수라고 생각하면됨
    if os.name == 'nt': # os.name = 현재의 컴퓨터 운영체제를 의미하며, (nt : 윈도우) (posix : 리눅스)
        return msvcrt.getch().decode('utf-8') # 윈도우 운영체제일 경우, 키보드 입력을 받아서 utf-8 형식으로 디코딩한 값을 반환
    fd = sys.stdin.fileno() # 자세한 설명 : https://stackoverflow.com/questions/32199552/what-is-sys-stdin-fileno-in-python
    tty.setraw(fd) # fileno = raw 모드로 전환, 참고로 fileno 는 스트림의 기본이 되는 file descriptor를 반환하는 함수
    rlist, _, _ = select.select([sys.stdin],[],[], 0.1) 
    # 시스템 호출에 관한 함수  
    # select.selct(읽을 준비, 쓰기 준비, 예외 조건 대기, 딜레이), sys.stdin = 인터프리터의 표준 입력에 해당
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # tcsetattr(fd, when, attr) : fd에 대한 tty 속성을 attr로 설정, 언제 속성이 변경될지 when 으로 설정하며, TCSADRAIN은 모든 출력을 전송한 후에 변경함
    return key # 읽어온 키값 반환

class Routh_Auto_Driving(Node):

    def __init__(self, w,h, MODE, SCV, TCV, GCV, DCV, LV, AV, Reps, PD):
        super().__init__('KIGAM')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.twist = Twist()
        self.i = 0
        self.turn_ctr = False
        self.straight_timer = self.create_timer(0.01, self.timer_callback)

        self.MODE = MODE
        self.w = float(w)
        self.h = int(h)
        self.SCV = int(SCV)
        self.TCV = int(TCV)
        self.GCV = int(GCV)
        self.DCV = int(DCV)
        self.LV = float(LV)
        self.AV = float(AV)
        self.PD = float(PD)
        self.TOT_REP = int(Reps)
        self.REP = 0
        self.weight = 0.06
        
        if(self.MODE == '1'):
            self.stop_count = float(self.w/(self.PD*10))
            self.SDCV = int(self.SCV/self.stop_count)
            self.SCV = 0
            self.count_value = 0

        self.REP1 = self.SCV + self.TCV + int(self.TCV * self.weight)
        self.REP2 = self.REP1 + self.GCV 
        self.REP3 = self.REP2 + self.TCV + int(self.TCV * self.weight)
        self.REP4 = self.REP3 + self.SCV
        self.REP5 = self.REP4 + self.TCV + int(self.TCV * self.weight)
        self.REP6 = self.REP5 + self.GCV
        self.REP7 = self.REP6 + self.TCV + int(self.TCV * self.weight)

    def timer_callback(self):
        if(self.MODE == '0'):
            if(self.REP % 2 == 0 and self.REP != self.TOT_REP):
                if(self.i < self.SCV): # S
                    self.set_Velocity(lx = self.LV, az=0.0)
                    self.i += 1
                elif(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az =self.AV)
                    self.i += 1
                else:
                    self.REP += 1
                    self.i = 0
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.REP % 2 == 1 and self.REP != self.TOT_REP):
                if(self.i < self.SCV): # S
                    self.set_Velocity(lx = self.LV, az=0.0)
                    self.i += 1
                elif(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = -self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az =-self.AV)
                    self.i += 1
                else:
                    self.REP += 1
                    self.i = 0
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.REP == self.TOT_REP):
                if(self.i < self.SCV):
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                else:
                    self.MODE = '2'
                    print(" >>> All Runs are Completed, Ctrl+C to Quit !!! <<< ")

        elif(self.MODE == '1'):
            if(self.count_value < self.stop_count and self.REP != self.TOT_REP):
                if(self.i < self.SDCV):
                    self.set_Velocity(lx = self.LV, az=0.0)
                    self.i += 1
                elif(self.i >= self.SDCV and self.i < self.SDCV + self.DCV):
                    self.set_Velocity()
                    self.i += 1
                else:
                    self.count_value += 1
                    self.i = 0
            elif(self.count_value >= self.stop_count and self.REP %2 == 0):
                if(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az =self.AV)
                    self.i += 1
                else:
                    self.count_value = 0
                    self.i = 0
                    self.REP += 1
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.count_value >= self.stop_count and self.REP %2 == 1):
                if(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = -self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az = -self.AV)
                    self.i += 1
                else:
                    self.count_value = 0
                    self.i = 0
                    self.REP += 1
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            if(self.REP == self.TOT_REP):
                if(self.count_value < self.stop_count):
                    if(self.i < self.SDCV):
                        self.set_Velocity(lx = self.LV, az=0.0)
                        self.i += 1
                    elif(self.i >= self.SDCV and self.i < self.SDCV + self.DCV):
                        self.set_Velocity()
                        self.i += 1
                    else:
                        self.count_value += 1
                        self.i = 0
                else:
                    self.set_Velocity()
                    self.MODE = '2'
                    print(" >>> All Runs are Completed, Ctrl+C to Quit !!! <<< ")
        else:
            self.MODE = '2'
            self.set_Velocity()

    def set_Velocity(self, lx=0.0, ly=0.0, lz=0.0, ax= 0.0, ay=0.0, az=0.0):
        self.twist.linear.x = lx
        self.twist.linear.y = ly
        self.twist.linear.z = lz

        self.twist.angular.x = ax
        self.twist.angular.y = ay
        self.twist.angular.z = az 

        self.velocity_pub.publish(self.twist)

def main(args=None):

    rclpy.init(args=args)

    input_count = 0

    print(msg)

    case = input("-- Driving Method [ 0 : Continuous, 1 : Partial ] : ")

    oper_key_w = 0
    oper_key_h = 0

    stop_time = 0

    gap = 0
    stop_dist = 0
    stop_delay = 0

    LV = 0
    AV = 0
    SCV = 0
    TCV = 0
    GCV = 0
    DCV = 0
    Repetitions = 0

    # oper_key list meaning = 0 : Scan area size (mm), 1 : Linear Velocity (m/s), 2 : Angular Velocity (deg/s) 
    if(case == '0'):
        while(input_count < 4):
            if(input_count == 0):
                oper_key_w = input("①  Scan Area Width - unit [mm] : ")
                oper_key_h = input("②  Scan Area Height - unit [mm] : ")
            elif(input_count == 1):
                LV = input("③  Linear Velocity (0 ~ 0.26) - unit [m/s] : ")
            elif(input_count == 2):
                AV = input("④  Angular Velocity, rotate per degree (0 ~ 90, Recommend : 30) - unit [degree] : ")
            elif(input_count == 3):
                gap = input("⑤  Path Spacing - unit [cm] : ")
            input_count += 1

    elif(case == '1'):
        while(input_count < 5):
            if(input_count == 0):
                oper_key_w = input("①  Scan Area Width - unit [mm] : ")
                oper_key_h = input("②  Scan Area Height - unit [mm] : ")
            elif(input_count == 1):
                LV = input("③  Linear Velocity (0 ~ 0.26) - unit [m/s] : ")
            elif(input_count == 2):
                AV = input("④  Angular Velocity, rotate per degree (0 ~ 90, Recommend : 30) - unit [degree] : ")
            elif(input_count == 3):
                gap = input("⑤  Path Spacing - unit [cm] : ")
            elif(input_count == 4):
                stop_dist = input("⑥  Pause Distance [cm] : ")
                stop_delay = input("⑦  Stop Delay [s] : ")
            input_count += 1

    else:
        print("Please Enter a Valid Number, 0 or 1")

    velocity = SCV
    SCV = float(oper_key_w)/(float(LV) * 0.01 * 1000)
    angle = float(AV)
    angle_to_rad = angle*(math.pi/180)
    TCV = 90/(int(angle) * 0.01)
    GCV = float(gap)/(float(LV) * 0.01 * 100)
    Repetitions = int(oper_key_h)/(int(gap)*10)

    DCV = float(stop_delay)/(0.01)

    if(case == '0'):
        print("\n:::::: Area : %.2f x %.2f mm, LV : %.3f m/s, AV : %f rad/s (%.1f degree/s), Gap : %.0f cm" %(float(oper_key_w), float(oper_key_h), float(LV), float(angle_to_rad), float(angle), float(gap)))
        print(":::::: ST Count Value : %.1f, TR Count Value : %.1f, SP Count Value : %.1f, Rep : %d" %(float(SCV), float(TCV), float(GCV), int(Repetitions)))
    elif(case == '1'):
        print("\n:::::: Area : %.2f x %.2f mm, LV : %.3f m/s, AV : %f rad/s (%.1f degree/s), Gap : %.0f cm, Pause Dist : %.0f cm" \
            %(float(oper_key_w), float(oper_key_h), float(LV), float(angle_to_rad), float(angle), float(gap), float(stop_dist)))
        print(":::::: ST Count Value : %.1f, TR Count Value : %.1f, SP Count Value : %.1f, Delay Count Value : %.1f, Rep : %d" %(float(SCV), float(TCV), float(GCV), float(DCV), int(Repetitions)))

    KIGAM_ROBOT = Routh_Auto_Driving(oper_key_w, oper_key_h, case, SCV, TCV, GCV, DCV, LV, angle_to_rad, Repetitions, stop_dist)

    while rclpy.ok():
        try:
            rclpy.spin(KIGAM_ROBOT)
            key = get_key(settings)
            if(key == '\x03'):
                KIGAM_ROBOT.set_Velocity()
                KIGAM_ROBOT.destroy_node()
                rclpy.shutdown()
                print("프로그램이 종료되었습니다.")
                break
        except KeyboardInterrupt as e:
            break
        except UnboundLocalError as e:
            break

if __name__ == '__main__':
    main()
