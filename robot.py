import pybullet as p
import math
from collections import namedtuple
import time
import numpy as np

class RobotBase(object):
    """
    The base class for robots
    """

    def __init__(self, pos, ori):
        """
        Arguments:
            pos: [x y z]
            ori: [r p y]

        Attributes:
            id: Int, the ID of the robot
            eef_id: Int, the ID of the End-Effector
            arm_num_dofs: Int, the number of DoFs of the arm
                i.e., the IK for the EE will consider the first `arm_num_dofs` controllable (non-Fixed) joints
            joints: List, a list of joint info
            controllable_joints: List of Ints, IDs for all controllable joints
            arm_controllable_joints: List of Ints, IDs for all controllable joints on the arm (that is, the first `arm_num_dofs` of controllable joints)

            ---
            For null-space IK
            ---
            arm_lower_limits: List, the lower limits for all controllable joints on the arm
            arm_upper_limits: List
            arm_joint_ranges: List
            arm_rest_poses: List, the rest position for all controllable joints on the arm

            gripper_range: List[Min, Max]
        """
        self.base_pos = pos
        self.base_ori = p.getQuaternionFromEuler(ori)

    def load(self):
        self.__init_robot__()
        self.__parse_joint_info__()
        self.__post_load__()
        print(self.joints)

    def step_simulation(self):
        raise RuntimeError('`step_simulation` method of RobotBase Class should be hooked by the environment.')

    def __parse_joint_info__(self):
        numJoints = p.getNumJoints(self.id)
        jointInfo = namedtuple('jointInfo', 
            ['id','name','type','damping','friction','lowerLimit','upperLimit','maxForce','maxVelocity','controllable'])
        self.joints = []
        self.controllable_joints = []
        for i in range(numJoints):
            info = p.getJointInfo(self.id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]  # JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED
            jointDamping = info[6]
            jointFriction = info[7]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = (jointType != p.JOINT_FIXED)
            if controllable:
                self.controllable_joints.append(jointID)
                p.setJointMotorControl2(self.id, jointID, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            info = jointInfo(jointID,jointName,jointType,jointDamping,jointFriction,jointLowerLimit,
                            jointUpperLimit,jointMaxForce,jointMaxVelocity,controllable)
            self.joints.append(info)

        assert len(self.controllable_joints) >= self.arm_num_dofs
        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]

        self.arm_lower_limits = [info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [info.upperLimit for info in self.joints if info.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [info.upperLimit - info.lowerLimit for info in self.joints if info.controllable][:self.arm_num_dofs]

    def __init_robot__(self):
        raise NotImplementedError
    
    def __post_load__(self):
        pass

    def reset(self):
        self.reset_arm()
        self.reset_gripper()

    def reset_arm(self):
        """
        reset to rest poses
        """
        for rest_pose, joint_id in zip(self.arm_rest_poses, self.arm_controllable_joints):
            p.resetJointState(self.id, joint_id, rest_pose)

        # Wait for a few steps
        for _ in range(10):
            self.step_simulation()

    def reset_gripper(self):
        self.open_gripper()

    def open_gripper(self):
        self.move_gripper(self.gripper_range[1])

    def close_gripper(self):
        self.move_gripper(self.gripper_range[0])

    # def move_ee(self, action, control_method):
    #     assert control_method in ('joint', 'end')
    #     if control_method == 'end':
    #         x, y, z, roll, pitch, yaw = action
    #         pos = (x, y, z)
    #         orn = p.getQuaternionFromEuler((roll, pitch, yaw))
    #         # joint_poses = p.calculateInverseKinematics(self.id, self.eef_id, pos, orn,
    #         #                                            self.arm_lower_limits, self.arm_upper_limits, self.arm_joint_ranges, self.arm_rest_poses,
    #         #                                            maxNumIterations=100, residualThreshold=1e-4)
    #
    #         # 我写的 第一版
    #         # joint_poses = p.calculateInverseKinematics(bodyUniqueId=self.id,
    #         #                                             endEffectorLinkIndex=self.eef_id,
    #         #                                             targetPosition=pos,
    #         #                                             targetOrientation=orn,
    #         #                                             lowerLimits=self.arm_lower_limits,
    #         #                                             upperLimits=self.arm_upper_limits,
    #         #                                             jointRanges=self.arm_joint_ranges,
    #         #                                             maxNumIterations=100,
    #         #                                             residualThreshold=1e-4,
    #         #                                             solver=0)
    #         # 我写的 第二版
    #         threshold = 1e-2
    #         max_iter = 200
    #         close_enough = False
    #         iter = 0
    #         # print('可控制的关节数量是：', len(self.arm_controllable_joints))
    #         # print(self.arm_controllable_joints)
    #         # joint_num = p.getNumJoints(self.id)
    #         # print('关节数量是：', joint_num)
    #         # for i in range(joint_num):
    #         #     print(p.getJointInfo(self.id, i))
    #         joint_poses = [0] * 12
    #         while (not close_enough and iter < max_iter):
    #             joint_poses = p.calculateInverseKinematics(self.id,
    #                                                        self.eef_id,
    #                                                        pos,
    #                                                        orn,
    #                                                        lowerLimits=self.arm_lower_limits,
    #                                                        upperLimits=self.arm_upper_limits,
    #                                                        jointRanges=self.arm_joint_ranges, )
    #             for i in range(len(self.arm_controllable_joints)):
    #                 p.resetJointState(self.id, i+1, joint_poses[i])
    #             link_state = p.getLinkState(self.id, self.eef_id)
    #             new_pose = link_state[4]
    #             import numpy as np
    #             diff = np.linalg.norm(np.array(pos) - np.array(new_pose))
    #             close_enough = diff < threshold
    #             iter += 1
    #
    #     elif control_method == 'joint':
    #         assert len(action) == self.arm_num_dofs
    #         joint_poses = action
    #     # arm
    #     for i, joint_id in enumerate(self.arm_controllable_joints):
    #         # p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, joint_poses[i],
    #         #                         force=self.joints[joint_id].maxForce, maxVelocity=self.joints[joint_id].maxVelocity)
    #         p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, joint_poses[i], maxVelocity=1)
    #         # p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, joint_poses[i],
    #         #                         force=1000, maxVelocity=self.joints[joint_id].maxVelocity)

    def move_ee(self, action, control_method, velocity):
        assert control_method in ('joint', 'end')
        if control_method == 'end':
            x, y, z, roll, pitch, yaw = action
            pos = (x, y, z)
            orn = p.getQuaternionFromEuler((roll, pitch, yaw))
            # print('角度范围是：')
            # print(self.arm_lower_limits)
            # print(self.arm_upper_limits)
            # print(self.arm_joint_ranges)

            self.arm_lower_limits = [-math.pi/2] * 6
            self.arm_upper_limits = [math.pi/2] * 6
            self.arm_joint_ranges = [math.pi] * 6
            # 初始IK计算
            joint_poses = p.calculateInverseKinematics(
                self.id, self.eef_id, pos, orn,
                lowerLimits=self.arm_lower_limits,
                upperLimits=self.arm_upper_limits,
                jointRanges=self.arm_joint_ranges,
                maxNumIterations=50,
            )

            # 应用初始关节角度
            for i, joint_id in enumerate(self.arm_controllable_joints):
                p.setJointMotorControl2(
                    self.id, joint_id, p.POSITION_CONTROL,
                    targetPosition=joint_poses[i],
                    force=100,
                    maxVelocity=velocity
                )

            # 迭代优化
            threshold = 1e-2
            max_iter = 200
            close_enough = False
            iter = 0

            while not close_enough and iter < max_iter:
                # 执行仿真步骤
                p.stepSimulation()
                time.sleep(1 / 240)

                # 获取当前末端位置
                link_state = p.getLinkState(self.id, self.eef_id)
                current_pos = link_state[0]

                # 计算误差
                diff = np.linalg.norm(np.array(pos) - np.array(current_pos))
                close_enough = diff < threshold

                # 可选：如果误差较大，重新计算IK并调整
                if not close_enough and iter < max_iter:
                    joint_poses = p.calculateInverseKinematics(
                        self.id, self.eef_id, pos, orn,
                        lowerLimits=self.arm_lower_limits,
                        upperLimits=self.arm_upper_limits,
                        jointRanges=self.arm_joint_ranges,
                        maxNumIterations=50,
                    )

                    for i, joint_id in enumerate(self.arm_controllable_joints):
                        p.setJointMotorControl2(
                            self.id, joint_id, p.POSITION_CONTROL,
                            targetPosition=joint_poses[i],
                            force=100,
                            maxVelocity=velocity
                        )

                iter += 1

        elif control_method == 'joint':
            # 关节空间控制保持不变
            for i, joint_id in enumerate(self.arm_controllable_joints):
                p.setJointMotorControl2(
                    self.id, joint_id, p.POSITION_CONTROL,
                    targetPosition=action[i],
                    force=50,
                    maxVelocity=velocity
                )
                p.stepSimulation()
                time.sleep(1 / 240)
    def move_gripper(self, open_length):
        raise NotImplementedError

    def get_joint_obs(self):
        positions = []
        velocities = []
        for joint_id in self.controllable_joints:
            pos, vel, _, _ = p.getJointState(self.id, joint_id)
            positions.append(pos)
            velocities.append(vel)
        ee_pos = p.getLinkState(self.id, self.eef_id)[0]
        return dict(positions=positions, velocities=velocities, ee_pos=ee_pos)


class Panda(RobotBase):
    def __init_robot__(self):
        # define the robot
        # see https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_robots/panda/panda_sim_grasp.py
        self.eef_id = 11
        self.arm_num_dofs = 7
        self.arm_rest_poses = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32]
        self.id = p.loadURDF('./robot_urdf/urdf/panda.urdf', self.base_pos, self.base_ori,
                             useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        self.gripper_range = [0, 0.04]
        # create a constraint to keep the fingers centered
        c = p.createConstraint(self.id,
                               9,
                               self.id,
                               10,
                               jointType=p.JOINT_GEAR,
                               jointAxis=[1, 0, 0],
                               parentFramePosition=[0, 0, 0],
                               childFramePosition=[0, 0, 0])
        p.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

    def move_gripper(self, open_length):
        assert self.gripper_range[0] <= open_length <= self.gripper_range[1]
        for i in [9, 10]:
            p.setJointMotorControl2(self.id, i, p.POSITION_CONTROL, open_length, force=100)


class UR5Robotiq85(RobotBase):
    def __init_robot__(self):
        self.eef_id = 7
        self.arm_num_dofs = 6
        self.arm_rest_poses = [1.488, -1.62, 1.918, -1.852,
                               -1.653, 0]
        # self.id = p.loadURDF('./robot_urdf/urdf/ur5_robotiq_85.urdf', self.base_pos, self.base_ori,
        #                      useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        self.id = p.loadURDF('robot_urdf/urdf/ur5_robotiq_85.urdf', self.base_pos, self.base_ori,
                             useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        self.gripper_range = [0, 0.085]

    def __post_load__(self):
        # To control the gripper
        mimic_parent_name = 'finger_joint'
        mimic_children_names = {'right_outer_knuckle_joint': 1,
                                'left_inner_knuckle_joint': 1,
                                'right_inner_knuckle_joint': 1,
                                'left_inner_finger_joint': -1,
                                'right_inner_finger_joint': -1}
        self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)

    def __setup_mimic_joints__(self, mimic_parent_name, mimic_children_names):
        self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
        self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if joint.name in mimic_children_names}

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = p.createConstraint(self.id, self.mimic_parent_id,
                                   self.id, joint_id,
                                   jointType=p.JOINT_GEAR,
                                   jointAxis=[0, 1, 0],
                                   parentFramePosition=[0, 0, 0],
                                   childFramePosition=[0, 0, 0])
            p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)  # Note: the mysterious `erp` is of EXTREME importance

    def move_gripper(self, open_length):
        # open_length = np.clip(open_length, *self.gripper_range)
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)  # angle calculation
        print('打开的角度是：', open_angle)
        # Control the mimic gripper joint(s)
        p.setJointMotorControl2(self.id, self.mimic_parent_id, p.POSITION_CONTROL, targetPosition=open_angle,
                                force=100, maxVelocity=self.joints[self.mimic_parent_id].maxVelocity)


class UR5Robotiq140(UR5Robotiq85):
    def __init_robot__(self):
        self.eef_id = 7
        self.arm_num_dofs = 6
        self.arm_rest_poses = [-1.5690622952052096, -1.5446774605904932, 1.343946009733127, -1.3708613585093699,
                               -1.5707970583733368, 0.0009377758247187636]
        self.id = p.loadURDF('./robot_urdf/urdf/ur5_robotiq_140.urdf', self.base_pos, self.base_ori,
                             useFixedBase=True, flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        self.gripper_range = [0, 0.085]
        # TODO: It's weird to use the same range and the same formula to calculate open_angle as Robotiq85.

    def __post_load__(self):
        mimic_parent_name = 'finger_joint'
        mimic_children_names = {'right_outer_knuckle_joint': -1,
                                'left_inner_knuckle_joint': -1,
                                'right_inner_knuckle_joint': -1,
                                'left_inner_finger_joint': 1,
                                'right_inner_finger_joint': 1}
        self.__setup_mimic_joints__(mimic_parent_name, mimic_children_names)
