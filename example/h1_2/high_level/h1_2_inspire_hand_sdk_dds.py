#控制H1—2的灵巧手
#2025.10.29
import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

#h1-2安装的因时一代手的默认消息
# def unitree_h12_msg_dds__InspireHandCmds_():
#     cmd_list = [MotorCmd_(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0) for _ in range(12)]
#     return MotorCmds_(cmds=cmd_list)
def unitree_h12_msg_dds__InspireHandCmds_():
    cmd_list = [
        MotorCmd_(
            0,          # mode
            0.0,        # q
            0.0,        # dq
            0.0,        # tau
            0.0,        # kp
            0.0,        # kd
            [0, 0, 0]   # reserve (must be a 3-element array)
        )
        for _ in range(12)
    ]
    return MotorCmds_(cmds=cmd_list)
class H1_2_HandsIndex:  
    #right hand
    RightPinky = 0
    RightRing = 1
    RightMiddle = 2
    RightIndex = 3
    RightThumbBend = 4
    RightThumbRotation = 5
    #left hand
    LeftPinky = 6
    LeftRing = 7
    LeftMiddle = 8
    LeftIndex = 9
    LeftThumbBend = 10
    LeftThumbRotation = 11
       

class H1HandController:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002   
        self.duration_ = 3.0   
        self.counter_ = 0
        self.update_mode_machine_ = True  # 添加这个缺失的变量

        self.hands_low_cmd = unitree_h12_msg_dds__InspireHandCmds_()
        self.hands_low_state = None

        self.crc = CRC()

        self.done = False
        
        self.kp_hand=np.array([100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 ])
        self.kd_hand=np.array([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 ])
        
        self.hand_joints = [H1_2_HandsIndex.RightPinky, 
                           H1_2_HandsIndex.RightRing, 
                           H1_2_HandsIndex.RightMiddle, 
                           H1_2_HandsIndex.RightIndex, 
                           H1_2_HandsIndex.RightThumbBend, 
                           H1_2_HandsIndex.RightThumbRotation, 
                           H1_2_HandsIndex.LeftPinky, 
                           H1_2_HandsIndex.LeftRing, 
                           H1_2_HandsIndex.LeftMiddle, 
                           H1_2_HandsIndex.LeftIndex, 
                           H1_2_HandsIndex.LeftThumbBend, 
                           H1_2_HandsIndex.LeftThumbRotation]
        
        # 连接状态标志
        self.is_connected = False
        self.state_received = False
        self.last_state_time = 0
        self.state_count = 0

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        
        # 创建发布器和订阅器
        try:
            self.hand_sdk_publisher = ChannelPublisher("rt/inspire/cmd", MotorCmds_)
            self.hand_sdk_publisher.Init()
            
            self.handstate_subscriber = ChannelSubscriber("rt/inspire/state", MotorStates_)
            self.handstate_subscriber.Init(self.HandStateHandler, 10)
            
            self.is_connected = True
            print("✓ DDS channels initialized successfully")
            print("  - Publishing to: rt/inspire/cmd")
            print("  - Subscribing to: rt/inspire/state")
        except Exception as e:
            print(f"✗ Failed to initialize DDS channels: {e}")
            self.is_connected = False

    def Start(self):
        if not self.is_connected:
            print("Cannot start: DDS not connected")
            return
            
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.hands_demo, name="control"
        )
        
        # 等待模式机更新
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()
            print("Control thread started")

    def HandStateHandler(self, msg: MotorStates_):
        """改进的状态处理函数"""
        self.hands_low_state = msg
        self.state_received = True
        self.last_state_time = time.time()
        self.state_count += 1
        
        # 每10次状态更新打印一次详细信息
        if self.state_count % 10 == 0:
            self.print_detailed_state()

    def print_detailed_state(self):
        """打印详细的手部状态信息"""
        if not self.hands_low_state:
            return
            
        print("\n" + "="*50)
        print("HAND STATE UPDATE")
        print("="*50)
        
        # 右手状态
        print("RIGHT HAND:")
        for i in range(6):
            state = self.hands_low_state.states[i]
            finger_names = ["Pinky", "Ring", "Middle", "Index", "ThumbBend", "ThumbRot"]
            print(f"  {finger_names[i]:10}: q={state.q:.3f}, tau={state.tau_est:.3f}, temp={state.temperature}")
        
        # 左手状态
        print("LEFT HAND:")
        for i in range(6, 12):
            state = self.hands_low_state.states[i]
            finger_names = ["Pinky", "Ring", "Middle", "Index", "ThumbBend", "ThumbRot"]
            print(f"  {finger_names[i-6]:10}: q={state.q:.3f}, tau={state.tau_est:.3f}, temp={state.temperature}")
        
        print("="*50)

    def send_hand_command(self, right_angles, left_angles):
        """发送手部控制命令
        
        Args:
            right_angles: 右手6个关节的角度列表 [0-1]
            left_angles: 左手6个关节的角度列表 [0-1]
        """
        if not self.is_connected:
            print("Cannot send command: DDS not connected")
            return False
            
        if len(right_angles) != 6 or len(left_angles) != 6:
            print("Error: Must provide exactly 6 angles for each hand")
            return False
            
        cmd = unitree_h12_msg_dds__InspireHandCmds_()
        
        # 设置右手命令
        for i in range(6):
            cmd.cmds[i].mode = 0
            cmd.cmds[i].q = float(right_angles[i])
            cmd.cmds[i].kp = self.kp_hand[i]
            cmd.cmds[i].kd = self.kd_hand[i]
            
        # 设置左手命令
        for i in range(6):
            cmd.cmds[i].mode = 0
            cmd.cmds[i+6].q = float(left_angles[i])
            cmd.cmds[i+6].kp = self.kp_hand[i+6]
            cmd.cmds[i+6].kd = self.kd_hand[i+6]
            
        try:
            self.hand_sdk_publisher.Write(cmd)
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False

    def send_test_command(self):
        """发送测试命令来验证通讯"""
        if not self.is_connected:
            print("DDS not connected")
            return False
            
        # 创建打开手的命令 (所有关节设为1.0)
        right_angles = [1.0] * 6  # 完全打开
        left_angles = [1.0] * 6   # 完全打开
        
        success = self.send_hand_command(right_angles, left_angles)
        if success:
            print("✓ Test command sent: OPEN hand")
        return success

    def send_close_command(self):
        """发送关闭手的命令"""
        if not self.is_connected:
            print("DDS not connected")
            return False
            
        # 创建关闭手的命令 (所有关节设为0.0)
        right_angles = [0.0] * 6  # 完全关闭
        left_angles = [0.0] * 6   # 完全关闭
        
        success = self.send_hand_command(right_angles, left_angles)
        if success:
            print("✓ Command sent: CLOSE hand")
        return success

    def send_preset_gesture(self, gesture_id):
        """发送预设手势"""
        gestures = {
            '0': {'right': [1.0, 1.0, 1.0, 1.0, 1.0, 0.2], 'left': [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]},  # 打开
            '1': {'right': [0.4, 0.5, 0.6, 0.7, 0.7, 0.1], 'left': [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]},  # 手势1
            '2': {'right': [0.5, 0.6, 0.7, 0.85, 0.9, 0.1], 'left': [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]}, # 手势2
            '3': {'right': [0.2, 0.2, 0.2, 0.2, 0.2, 0.6], 'left': [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]},  # 手势3
            '4': {'right': [0.0, 0.0, 0.0, 0.0, 0.3, 0.5], 'left': [0.0, 0.0, 0.0, 0.0, 0.3, 0.5]}   # 手势4
        }
        
        if gesture_id in gestures:
            gesture = gestures[gesture_id]
            success = self.send_hand_command(gesture['right'], gesture['left'])
            if success:
                print(f"✓ Preset gesture {gesture_id} sent")
            return success
        else:
            print(f"Unknown gesture ID: {gesture_id}")
            return False

    def hands_demo(self):
        """演示函数：每5秒切换开合状态"""
        if not self.is_connected:
            return
            
        # 每5秒切换一次状态
        if self.counter_ % (int(5.0 / self.control_dt_)) == 0:
            state = (self.counter_ // int(5.0 / self.control_dt_)) % 2
            
            if state == 0:  # 关闭
                self.send_close_command()
            else:  # 打开
                self.send_test_command()
                
        self.counter_ += 1

    def check_connection_status(self):
        """检查DDS连接状态"""
        current_time = time.time()
        
        if not self.is_connected:
            return "NOT_CONNECTED", "DDS channels not initialized"
        
        if not self.state_received:
            return "CONNECTED_NO_DATA", "Connected but no state data received"
        
        time_since_last_state = current_time - self.last_state_time
        if time_since_last_state > 2.0:  # 2秒没有收到数据认为断开
            return "DISCONNECTED", f"No state data for {time_since_last_state:.1f}s"
        
        return "FULLY_CONNECTED", f"Receiving states ({self.state_count} messages)"

    def print_connection_status(self):
        """打印连接状态"""
        status, message = self.check_connection_status()
        status_symbols = {
            "NOT_CONNECTED": "✗",
            "CONNECTED_NO_DATA": "⚠", 
            "DISCONNECTED": "↯",
            "FULLY_CONNECTED": "✓"
        }
        symbol = status_symbols.get(status, "?")
        print(f"{symbol} Connection Status: {status} - {message}")

    def interactive_test(self):
        """交互式测试功能"""
        if not self.is_connected:
            print("Cannot start interactive test: DDS not connected")
            return
            
        print("\n" + "="*60)
        print("H1-2 HAND INTERACTIVE TEST")
        print("="*60)
        print("Commands:")
        print("  o - Open hand")
        print("  c - Close hand") 
        print("  0-4 - Preset gestures")
        print("  s - Show connection status")
        print("  p - Print current state")
        print("  q - Quit interactive test")
        print("="*60)
        
        while True:
            try:
                command = input("\nEnter command: ").strip().lower()
                
                if command == 'q':
                    print("Exiting interactive test")
                    break
                elif command == 'o':
                    self.send_test_command()
                elif command == 'c':
                    self.send_close_command()
                elif command in ['0', '1', '2', '3', '4']:
                    self.send_preset_gesture(command)
                elif command == 's':
                    self.print_connection_status()
                elif command == 'p':
                    if self.hands_low_state:
                        self.print_detailed_state()
                    else:
                        print("No state data available")
                else:
                    print("Unknown command. Available: o, c, 0-4, s, p, q")
                    
            except KeyboardInterrupt:
                print("\nExiting interactive test")
                break
            except Exception as e:
                print(f"Error in interactive test: {e}")

if __name__ == '__main__':
    print("H1-2 Hand DDS Communication Test")
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    
    # 初始化DDS
    if len(sys.argv) > 1:
        print(f"Using network interface: {sys.argv[1]}")
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        print("Using default network interface")
        ChannelFactoryInitialize(0)

    # 创建控制器实例
    h1hand = H1HandController()
    h1hand.Init()
    
    # 检查连接状态
    h1hand.print_connection_status()
    
    # 发送初始测试命令
    time.sleep(1)
    print("\nSending initial test command...")
    h1hand.send_test_command()
    print("\ninitial test command sended successfully...")
    
    # 启动控制线程
    h1hand.Start()
    
    # 等待几秒接收状态
    print("\nWaiting for state data...")
    for i in range(5):
        time.sleep(1)
        if h1hand.state_received:
            print("✓ Started receiving hand states")
            break
        else:
            print(f"  Waiting... {i+1}/5")
    
    # 进入交互式测试
    h1hand.interactive_test()
    
    # 退出前关闭手
    print("\nClosing hand before exit...")
    h1hand.send_close_command()
    time.sleep(1)
    
    print("Program finished")