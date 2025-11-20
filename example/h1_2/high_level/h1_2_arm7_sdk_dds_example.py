#控制H1—2上半身的脚本
#2025.10.29
import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import numpy as np

kPi = 3.141592654
kPi_2 = 1.57079632
# H1-2 电机总数
H1_2_NUM_MOTOR = 27

class H1_2_JointIndex:
    # legs
    LeftHipYaw = 0
    LeftHipPitch = 1
    LeftHipRoll = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipYaw = 6
    RightHipPitch = 7
    RightHipRoll = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    # torso
    WaistYaw = 12
    # arms
    LeftShoulderPitch = 13
    LeftShoulderRoll = 14
    LeftShoulderYaw = 15
    LeftElbow = 16
    LeftWristRoll = 17
    LeftWristPitch = 18
    LeftWristYaw = 19

    RightShoulderPitch = 20
    RightShoulderRoll = 21
    RightShoulderYaw = 22
    RightElbow = 23
    RightWristRoll = 24
    RightWristPitch = 25
    RightWristYaw = 26

    kNotUsedJoint = 27
    kNotUsedJoint1 = 28
    kNotUsedJoint2 = 29
    kNotUsedJoint3 = 30
    kNotUsedJoint4 = 31
    kNotUsedJoint5 = 32
    kNotUsedJoint6 = 33
    kNotUsedJoint7 = 34

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  
        self.duration_ = 3.0   
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.done = False

        self.kp=np.array([250, 250, 200, 250, 100, 100, 100, 
                          250, 250, 200, 250, 100, 100, 100, 
                          200 ])
        self.kd=np.array([25.0, 25.0, 20, 25, 10, 10, 10, 
                          25.0, 25.0, 20, 25, 10, 10, 10, 
                          20.0])
        self.dq = 0.
        self.tau_ff = 0.

        
        
        self.first_update_low_state = False

        self.done = False

        self.target_pos = [
            0., kPi_2,  0., kPi_2, 0., 0., 0.,
            0., -kPi_2, 0., kPi_2, 0., 0., 0., 
            0
        ]

        self.arm_joints = [H1_2_JointIndex.LeftShoulderPitch,
                           H1_2_JointIndex.LeftShoulderRoll, 
                           H1_2_JointIndex.LeftShoulderYaw, 
                           H1_2_JointIndex.LeftElbow, 
                           H1_2_JointIndex.LeftWristRoll, 
                           H1_2_JointIndex.LeftWristPitch, 
                           H1_2_JointIndex.LeftWristYaw, 
                           
                           H1_2_JointIndex.RightShoulderPitch, 
                           H1_2_JointIndex.RightShoulderRoll, 
                           H1_2_JointIndex.RightShoulderYaw, 
                           H1_2_JointIndex.RightElbow, 
                           H1_2_JointIndex.RightWristRoll, 
                           H1_2_JointIndex.RightWristPitch, 
                           H1_2_JointIndex.RightWristYaw,
                           #多加一个腰部，表示上半身关节索引
                           H1_2_JointIndex.WaistYaw]
        
    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        
        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.dual_arm_demo, name="control"
        )
        while self.update_mode_machine_ == False:
            time.sleep(1)

        if self.update_mode_machine_ == True:
            self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            print(self.low_state.imu_state.rpy)

    def dual_arm_demo(self):
        #双臂控制的简单实现
        self.time_ += self.control_dt_
        self.low_cmd.mode_pr = Mode.PR
        self.low_cmd.mode_machine = self.mode_machine_
        if self.time_ < self.duration_ :
          # [Stage 1]: set robot to zero posture
          self.low_cmd.motor_cmd[H1_2_JointIndex.kNotUsedJoint].q =  1 # 1:Enable arm_sdk, 0:Disable arm_sdk
          for i,joint in enumerate(self.arm_joints):
            ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[joint].mode =  1 # 1:Enable, 0:Disable
            self.low_cmd.motor_cmd[joint].tau = 0. 
            self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q 
            self.low_cmd.motor_cmd[joint].dq = 0. 
            self.low_cmd.motor_cmd[joint].kp = self.kp[i]
            self.low_cmd.motor_cmd[joint].kd = self.kd[i]
            print("set robot to zero posture")
        elif self.time_ < self.duration_ * 3 :
          # [Stage 2]: lift arms up
          for i,joint in enumerate(self.arm_joints):
            ratio = np.clip((self.time_ - self.duration_) / (self.duration_ * 2), 0.0, 1.0)              
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[joint].mode =  1 # 1:Enable, 0:Disable
            self.low_cmd.motor_cmd[joint].tau = 0. 
            self.low_cmd.motor_cmd[joint].q = ratio * self.target_pos[i] + (1.0 - ratio) * self.low_state.motor_state[joint].q 
            self.low_cmd.motor_cmd[joint].dq = 0. 
            self.low_cmd.motor_cmd[joint].kp = self.kp[i]
            self.low_cmd.motor_cmd[joint].kd = self.kd[i]
            print("lift arms up")
        elif self.time_ < self.duration_ * 6 :
          # [Stage 3]: set robot back to zero posture
          for i,joint in enumerate(self.arm_joints):
            ratio = np.clip((self.time_ - self.duration_*3) / (self.duration_ * 3), 0.0, 1.0) 
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[joint].mode =  1 # 1:Enable, 0:Disable
            self.low_cmd.motor_cmd[joint].tau = 0. 
            self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
            self.low_cmd.motor_cmd[joint].dq = 0. 
            self.low_cmd.motor_cmd[joint].kp = self.kp[i]
            self.low_cmd.motor_cmd[joint].kd = self.kd[i]
            print("set robot back to zero posture")  
        else:
           self.done = True
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.arm_sdk_publisher.Write(self.low_cmd)


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:        
        time.sleep(1)
        if custom.done: 
           print("Done!")
           sys.exit(-1)   