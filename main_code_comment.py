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
        self.i = 0 # Timer Count 를 저장하는 변수
        self.turn_ctr = False
        self.straight_timer = self.create_timer(0.01, self.timer_callback)

        self.MODE = MODE # 0 혹은 1 을 입력받아서 어떤 방식으로 주행할건지를 결정하는 변수
        self.w = float(w) # 사각형의 width
        self.h = int(h) # 사각형의 height 
        
        # 아래에서 설명하는 Timer Count 는 self.i 변수와 같다고 보면된다.
        
        self.SCV = int(SCV) 
        # 직진을 위해서 얼마만큼의 Timer Count 가 필요한지 저장하는 변수 (Straight Count Value 준말)
        # 예를 들어 1000mm 를 0.1m/s 로 간다고 하면, 0.01초의 타이머가 1000번 Tick 해야하므로, SCV = 1000이 된다.
        
        self.TCV = int(TCV)
        # 회전을 위해서 얼마만큼의 Timer Count 가 필요한지 저장하는 변수 (Turn Count Value 준말)
        # 예를 들어 90도를 회전하기 위해 초당 30도씩 회전하면, 총 3초를 회전해야하므로 0.01초의 타이머가 300번 Tick 해야하므로 TCV = 300이 된다. 
        
        self.GCV = int(GCV)
        # 직진간의 간격 주행을 위해 얼마만큼의 Timer Count 가 필요한지 저장하는 변수 (Gap Count Value 준말)
        # 예를 들어 Gap = 10cm 이고 0.1m/s 속도로 간다고 하면, 0.01 초의 타이머가 100번 Tick 해야하므로, GCV = 100이 된다.
        
        self.DCV = int(DCV)
        # Mode = 1 에서 잠시 멈추는 구간에서, 멈추는 시간동안 몇번의 Timer Count 가 필요한지 저장하는 변수 (Delay Count Value 준말)
        # 예를 들어, 10초 동안 정지를 한다고 하면, 0.01초 타이머가 1000번 Tick 해야하므로 DCV = 1000이 된다.
        
        self.LV = float(LV) # 입력받은 Linear Velocity
        self.AV = float(AV) # 입력받은 Degree per second 를 rad/s 로 변환하여 얻은 값
        self.PD = float(PD) # 입력받은 Pause Distance 로 정지와 정지 사이의 거리 (Mode = 1 에서)
        
        self.TOT_REP = int(Reps) # 전체 루틴의 반복 수를 저장하는 변수
        self.REP = 0 # 현재 루틴 횟수를 저장하는 변수
        
        self.weight = 0.06 # 로봇의 각속도 회전에 가중치를 부여하기 위한 변수
        self.stop_ctr = 1 # 초기에 정지상태를 제어하기 위한 변수 (Mode = 1 에서)
        
        if(self.MODE == '1'): # 만약 Partial 모드라면
            self.stop_count = float(self.w/(self.PD*10))
            # width 를 Pause Distance 로 몇번 가야하는지를 저장하는 변수 
            # 예를 들어, width = 1000mm , PD = 10cm 라면, stop_count = 10 ( 총 10번을 멈췄다가 가야함 )
            
            self.SDCV = int(self.SCV/self.stop_count)
            # 위의 stop_count 당 얼마의 거리를 가야하는지 Timer Count 를 저장하는 변수 (Short Distance Count Value 준말)
            # SCV = 1000 이라면, SDCV = 100 이 된다.  
            
            self.SCV = 0
            self.count_value = 0 # 현재 정지 횟수를 저장하는 변수

        self.REP1 = self.SCV + self.TCV + int(self.TCV * self.weight)  # 직진 + 회전 + 가중치값 까지 필요한 Timer Count 수
        self.REP2 = self.REP1 + self.GCV                               # 이전 REP1 에서 짧은거리 직진 까지 필요한 Timer Count 수
        self.REP3 = self.REP2 + self.TCV + int(self.TCV * self.weight) # REP2 에서 다시 회전을 하기까지 필요한 Timer Count 수
        self.REP4 = self.REP3 + self.SCV                               # 위 3개 과정을 반복
        self.REP5 = self.REP4 + self.TCV + int(self.TCV * self.weight)
        self.REP6 = self.REP5 + self.GCV
        self.REP7 = self.REP6 + self.TCV + int(self.TCV * self.weight)

    def timer_callback(self): # 0.01 초의 Tick 마다 실행되는 함수, self.i = 0.01 초 마다 증가함
        if(self.MODE == '0'): # 0 번 모드에서 
            if(self.REP % 2 == 0 and self.REP != self.TOT_REP): # 직진 + 반시계 회전 + 짧은거리 직진 + 반시계 회전 까지의 루틴
                if(self.i < self.SCV): # i 변수가 SCV 에 도달할 때 까지 해당 조건문을 반복 (장거리 직진)
                    self.set_Velocity(lx = self.LV, az=0.0) # 입력받은 Linear Velocity 변수를 로봇에게 설정 및 전달
                    self.i += 1 # Count 증가
                elif(self.i >= self.SCV and self.i < self.REP1): # i 변수가 TCV(회전) 값 까지 도달할 때 까지 해당 조건문 반복 (반시계 회전)
                    self.set_Velocity(lx = 0.0, az = self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # i 변수가 GCV 값 까지 도달할 때 까지 해당 조건문 반복 (짧은거리 직진)
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # i 변수가 TCV(회전) 값 까지 도달할 때 까지 해당 조건문 반복 (반시계 회전)
                    self.set_Velocity(lx= 0.0, az =self.AV)
                    self.i += 1
                else: # 한 루틴이 끝났을 경우, REP 변수를 증가시켜서 현재 진행상황을 알리며, i 를 0으로 초기화시킴
                    self.REP += 1
                    self.i = 0
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.REP % 2 == 1 and self.REP != self.TOT_REP): # 직진 + 시계 회전 + 짧은거리 직진 + 시계 회전 까지의 루틴
                if(self.i < self.SCV): # S
                    self.set_Velocity(lx = self.LV, az=0.0)
                    self.i += 1
                elif(self.i >= self.SCV and self.i < self.REP1): 
                    self.set_Velocity(lx = 0.0, az = -self.AV) # 위 루틴과 같으나, 시계방향 회전이므로 각속도 전달시 - 가 붙어있음
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): 
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): 
                    self.set_Velocity(lx= 0.0, az =-self.AV)
                    self.i += 1
                else:
                    self.REP += 1
                    self.i = 0
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.REP == self.TOT_REP): # 최종적으로 가야하는 height 길이 만큼 도달하게 될 경우
                if(self.i < self.SCV): # 장거리 직진이 필요하므로 수행
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                else: # 직진이 끝나면, MODE 변수를 2로 바꾸어 프로그램을 종료함
                    self.MODE = '2'
                    print(" >>> All Runs are Completed, Ctrl+C to Quit !!! <<< ")

        elif(self.MODE == '1'): # 1 번 모드에서 
            if(self.stop_ctr == 1): # 초기에 일정 시간만큼 정지 후 출발하기 위한 제어 변수 stop_ctr = 1 일경우
                if(self.i < self.DCV): # Delay Count Value 에 i 가 도달할 때 까지 반복 
                    self.set_Velocity() # 아무 값도 전달하지 않으면 정지
                    self.i += 1 # Count Value 증가
                else:
                    self.stop_ctr = 0 # 최초 정지가 끝나면 제어 변수를 0 으로 바꾸어 직진 수행
                    self.i = 0

            if(self.count_value < self.stop_count and self.REP != self.TOT_REP and self.stop_ctr == 0): # 직진 및 정지를 반복하는 루틴 
                # 만약 직진 및 정지 횟수인 count_value 가 총 횟수인 stop_count 와 같아지면 조건문을 빠져나감
                if(self.i < self.SDCV): 
                    self.set_Velocity(lx = self.LV, az=0.0)
                    self.i += 1
                elif(self.i >= self.SDCV and self.i < self.SDCV + self.DCV):
                    self.set_Velocity()
                    self.i += 1
                else: # 정지 및 직진 1회가 반복되면
                    self.count_value += 1 # count_value 값을 1 증가시킴
                    self.i = 0 # i = 0 으로 초기화
            elif(self.count_value >= self.stop_count and self.REP %2 == 0 and self.stop_ctr == 0): # MODE 0 에서와 같이 반시계 회전 + 짧은거리 직진 + 반시계 회전 까지의 루틴
                if(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az =self.AV)
                    self.i += 1
                elif(self.i >= self.REP3 and self.i < self.REP3 + self.DCV):
                    self.set_Velocity()
                    self.i += 1
                else:
                    self.count_value = 0
                    self.i = 0
                    self.REP += 1
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            elif(self.count_value >= self.stop_count and self.REP %2 == 1): # MODE 0 에서와 같이 시계 회전 + 짧은거리 직진 + 시계 회전 까지의 루틴
                if(self.i >= self.SCV and self.i < self.REP1): # T
                    self.set_Velocity(lx = 0.0, az = -self.AV)
                    self.i += 1
                elif(self.i >= self.REP1 and self.i < self.REP2): # SS
                    self.set_Velocity(lx=self.LV, az =0.0)
                    self.i += 1
                elif(self.i >= self.REP2 and self.i < self.REP3): # T
                    self.set_Velocity(lx= 0.0, az = -self.AV)
                    self.i += 1
                elif(self.i >= self.REP3 and self.i < self.REP3 + self.DCV):
                    self.set_Velocity()
                    self.i += 1
                else:
                    self.count_value = 0
                    self.i = 0
                    self.REP += 1
                    print("------ Total Repetition : %s / %s" %(self.REP, self.TOT_REP))

            if(self.REP == self.TOT_REP): # 최종적으로 가야하는 height 길이 만큼 도달하게 될 경우
                if(self.count_value < self.stop_count): # MODE 0 에서와는 다르게, 직진 및 정지가 필요하므로 해당 부분을 조건문으로 정의
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
