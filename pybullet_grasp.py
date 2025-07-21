"""
Pybullet + GraspNet实现机械臂夹爪位姿估计，可以用键盘控制连续的多个物体抓取。
Author: Neluland DuckZ
Date: 2025-06-18
"""

import pybullet as p
import pybullet_data
import random
import time
import math
import numpy as np
import open3d as o3d
import threading
import sys
import copy
from typing import List, Dict, Tuple, Optional
from scipy.spatial.transform import Rotation as R
from graspnet.graspnet import GraspBaseline
from robot import UR5Robotiq85, Panda


class GraspingSimulator:
    """机器人抓取仿真系统"""

    def __init__(self):
        # 初始化仿真环境
        self._init_simulation_environment()

        # 初始化机器人
        self.robot = UR5Robotiq85((0.2, -0.7, 0.6), (0, 0, 0))
        self.robot.load()
        self.robot.step_simulation = p.stepSimulation
        self.robot.reset()

        # 初始化摄像头参数
        self._init_camera()

        # 创建投放区域
        self.drop_zone_id = self._create_drop_zone(
            position=[-0.4, -0.2, 0.62],
            size=[0.5, 0.5]
        )

        # 物体配置
        self.available_objects = [
            "ycb_objects/YcbGelatinBox/model.urdf",
            "ycb_objects/YcbMasterChefCan/model.urdf",
            "ycb_objects/YcbMustardBottle/model.urdf",
            "ycb_objects/YcbPear/model.urdf",
            "ycb_objects/YcbPottedMeatCan/model.urdf",
            "ycb_objects/YcbTennisBall/model.urdf",
        ]

        self.object_masses = {
            "YcbGelatinBox": 1.0,
            "YcbMasterChefCan": 0.3,
            "YcbMustardBottle": 0.4,
            "YcbPear": 0.2,
            "YcbPottedMeatCan": 0.3,
            "YcbTennisBall": 0.057,
        }

        self.scale_mapping = {
            "YcbMustardBottle": 0.8,
            "YcbGelatinBox": 0.8,
            "YcbMasterChefCan": 0.8,
            "YcbPottedMeatCan": 0.8
        }

        # 初始化抓取网络
        self.grasp_net = GraspBaseline()

        # 存储场景中的物体ID
        self.object_ids: List[int] = []

        # 键盘控制标志
        self.grasp_requested = False
        self.quit_requested = False
        self.reset_requested = False
        self.add_objects_requested = False

        # 检测操作系统
        self.use_windows = self._detect_os()

        # 初始化坐标变换矩阵
        self._init_transform_matrices()

    def _init_simulation_environment(self) -> None:
        """初始化PyBullet仿真环境"""
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.loadURDF("table/table.urdf", basePosition=[0, 0, 0.0], useFixedBase=True)
        p.setGravity(0, 0, -10)

        # 设置调试可视化
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=2.7,
            cameraYaw=63,
            cameraPitch=-44,
            cameraTargetPosition=[0, 0, 0]
        )

    def _init_camera(self) -> None:
        """初始化摄像头参数"""
        self.width = 640
        self.height = 480
        self.fov = 80
        self.aspect = self.width / self.height
        self.near = 0.02
        self.far = 1

        self.view_matrix = p.computeViewMatrix(
            cameraEyePosition=[0.23, -0.2, 1.0],
            cameraTargetPosition=[0.23, 0.05, 0.3],
            cameraUpVector=[0, 1, 0]
        )

        self.projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.aspect,
            nearVal=self.near,
            farVal=self.far
        )

    def _init_transform_matrices(self) -> None:
        """初始化坐标变换矩阵"""
        # pybullet camera 到 graspnet camera 坐标变换
        self.trans_camera = np.array([
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ])

        # pybullet camera to world
        self.trans_camera_world = np.asarray(self.view_matrix).reshape([4, 4], order="F")

    def _create_drop_zone(self, position: List[float], size: List[float]) -> int:
        """
        创建投放区域底板

        Args:
            position: 底板位置 [x, y, z]
            size: 底板尺寸 [width, height]

        Returns:
            投放区域的物体ID
        """
        # 创建碰撞形状
        collision_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[size[0] / 2, size[1] / 2, 0.005]
        )

        # 创建视觉形状（绿色半透明）
        visual_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size[0] / 2, size[1] / 2, 0.005],
            rgbaColor=[0.2, 0.8, 0.2, 0.8]
        )

        # 创建多体对象
        drop_zone_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=[position[0], position[1], position[2] + 0.005]
        )

        # 设置为完全固定
        p.changeDynamics(drop_zone_id, -1, mass=0)
        print(f"投放区域已创建在位置: {position}")

        return drop_zone_id

    def _detect_os(self) -> bool:
        """检测操作系统类型"""
        try:
            import msvcrt  # Windows
            return True
        except ImportError:
            try:
                import termios  # Linux/Mac
                return False
            except ImportError:
                print("错误：无法导入按键检测模块")
                sys.exit(1)

    def add_objects(self, num_objects: int = 5) -> None:
        """
        向场景中添加随机物体

        Args:
            num_objects: 要添加的物体数量
        """
        print(f"\n=== 正在添加 {num_objects} 个物体 ===")

        for i in range(num_objects):
            # 随机选择物体
            object_urdf = random.choice(self.available_objects)
            object_name = object_urdf.split('/')[-2]
            mass = self.object_masses.get(object_name, 0.5)
            scaling = self.scale_mapping.get(object_name, 1.0)

            # 随机生成姿态
            orientation = p.getQuaternionFromEuler([
                random.uniform(-np.pi / 4, np.pi / 4),
                random.uniform(-np.pi / 4, np.pi / 4),
                random.uniform(-np.pi, np.pi)
            ])

            # 寻找合适的位置（避免重叠）
            position = self._find_valid_position()
            if not position:
                print(f"无法为物体 {i + 1} 找到合适位置，跳过")
                continue

            # 加载物体
            try:
                object_id = p.loadURDF(
                    object_urdf,
                    basePosition=position,
                    baseOrientation=orientation,
                    globalScaling=scaling
                )

                # 设置物理属性
                p.changeDynamics(
                    object_id,
                    -1,
                    mass=mass,
                    lateralFriction=1.0,
                    restitution=0.01,
                    linearDamping=0.04,
                    angularDamping=0.04
                )

                self.object_ids.append(object_id)
                print(f"成功加载物体 {i + 1}: {object_name} 在位置 {position}")

            except Exception as e:
                print(f"无法加载物体 {object_urdf}: {e}")

        print(f"=== 共加载了 {len(self.object_ids)} 个物体 ===\n")

    def _find_valid_position(self, max_attempts: int = 20) -> Optional[List[float]]:
        """
        寻找一个不与现有物体重叠的随机位置

        Args:
            max_attempts: 最大尝试次数

        Returns:
            有效的位置坐标 [x, y, z]，或None（如果找不到）
        """
        for _ in range(max_attempts):
            # 生成随机位置
            position = [
                random.uniform(-0.1, 0.5),
                random.uniform(-0.3, 0.05),
                0.75 + random.uniform(0, 0.1)
            ]

            # 检查与现有物体的距离
            too_close = False
            for existing_id in self.object_ids:
                existing_pos = p.getBasePositionAndOrientation(existing_id)[0]
                distance = np.linalg.norm(np.array(position[:2]) - np.array(existing_pos[:2]))
                if distance < 0.12:  # 最小距离12cm
                    too_close = True
                    break

            if not too_close:
                return position

        return None

    def get_point_cloud(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        获取当前场景的点云数据

        Returns:
            点云坐标数组, 深度图像, 彩色图像
        """
        print("正在获取点云数据...")

        # 获取摄像头图像
        image_arr = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix
        )

        depth = image_arr[3]  # 深度图
        color = image_arr[2]  # 彩色图

        # 构建从像素坐标到世界坐标的变换矩阵
        proj_mat = np.asarray(self.projection_matrix).reshape([4, 4], order="F")
        view_mat = np.eye(4)
        tran_pix_world = np.linalg.inv(np.matmul(proj_mat, view_mat))

        # 生成像素网格
        y, x = np.mgrid[-1:1:2 / self.height, -1:1:2 / self.width]
        y *= -1.0
        x, y, z = x.reshape(-1), y.reshape(-1), depth.reshape(-1)
        h = np.ones_like(z)

        # 过滤无效深度
        pixels = np.stack([x, y, z, h], axis=1)
        pixels = pixels[z < 0.99]  # 排除无穷远深度
        pixels[:, 2] = 2 * pixels[:, 2] - 1  # 归一化深度

        # 转换到世界坐标
        points = np.matmul(tran_pix_world, pixels.T).T
        points /= points[:, 3:4]  # 齐次坐标转换

        return points[:, :3], depth, color

    def get_grasp_world(self, grasp) -> np.ndarray:
        """
        将抓取姿态从摄像头坐标系转换到世界坐标系

        Args:
            grasp: 抓取姿态对象

        Returns:
            世界坐标系下的4x4变换矩阵
        """
        # 构建抓取姿态矩阵
        grasp_trans = np.eye(4)
        grasp_trans[:3, :3] = grasp.rotation_matrix
        grasp_trans[:3, -1] = grasp.translation

        # 转换到世界坐标系
        grasp_trans_world = np.linalg.inv(self.trans_camera_world).dot(
            np.linalg.inv(self.trans_camera).dot(grasp_trans)
        )

        return grasp_trans_world

    def perform_grasp(self) -> bool:
        """
        执行一次完整的抓取流程

        Returns:
            抓取是否成功
        """
        print("\n=== 开始抓取流程 ===")

        # 获取点云数据
        points, _, _ = self.get_point_cloud()

        # 转换为Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 运行抓取检测
        print("运行抓取检测...")
        grasps = self.grasp_net.run(copy.deepcopy(pcd).transform(self.trans_camera), vis=False)

        if len(grasps) == 0:
            print("未找到有效的抓取点！")
            return False

        # 应用非极大值抑制并排序
        grasps.nms()
        grasps.sort_by_score()
        best_grasp = grasps[0]

        print(f"找到最佳抓取点，得分: {best_grasp.score:.4f}")

        # 获取世界坐标系下的抓取姿态
        grasp_world = self.get_grasp_world(best_grasp)

        # 计算预抓取位置（沿x轴后退10cm）
        trans_x_neg = np.eye(4)
        trans_x_neg[0, -1] = -0.1
        pre_grasp_world = grasp_world.dot(trans_x_neg)

        # 转换为欧拉角
        rot = R.from_matrix(pre_grasp_world[:3, :3])
        euler = rot.as_euler('xyz')
        print(f"抓取姿态欧拉角: {euler}")

        try:
            # 1. 移动到预抓取位置（抬高10cm）
            pre_pos = pre_grasp_world[:3, -1].tolist()
            pre_pos[-1] += 0.1
            print(f"移动到预抓取位置: {pre_pos}")
            self.robot.move_ee(pre_pos + euler.tolist(), 'end', velocity=0.8)
            self._simulate_steps(200)

            # 打印关节状态
            self._print_joint_states()

            # 2. 打开夹爪
            print("打开夹爪...")
            self.robot.open_gripper()
            self._simulate_steps(60)

            # 3. 移动到抓取位置
            pre_pos[-1] -= 0.12
            print(f"移动到抓取位置: {pre_pos}")
            self.robot.move_ee(pre_pos + euler.tolist(), 'end', velocity=0.2)
            self._simulate_steps(100)

            # 4. 闭合夹爪
            print("闭合夹爪...")
            self.robot.move_gripper(0)
            self._simulate_steps(200)

            # 5. 提起物体
            print("提起物体...")
            lift_pos = grasp_world[:3, -1].copy()
            lift_pos[2] += 0.3  # 抬高30cm
            self.robot.move_ee(lift_pos.tolist() + euler.tolist(), 'end', velocity=1.2)
            self._simulate_steps(200)

            # 6. 移动到投放区域上方
            print("移动到投放区域...")

            # 中间位置过渡
            intermediate_pos = [-0.1, -0.1, 0.62 + 0.3]
            self.robot.move_ee(intermediate_pos + [0, math.pi / 2, 0], 'end', velocity=1.5)
            self._simulate_steps(50)

            # 目标位置
            target_pos = [-0.4, -0.3, 0.62 + 0.3]
            self.robot.move_ee(target_pos + [0, math.pi / 2, 0], 'end', velocity=1.5)
            self._simulate_steps(50)

            # 降低高度
            target_pos[2] -= 0.1
            self.robot.move_ee(target_pos + [0, math.pi / 2, 0], 'end', velocity=0.8)
            self._simulate_steps(100)

            # 7. 释放物体
            print("释放物体...")
            self.robot.open_gripper()
            self._simulate_steps(100)

            # 8. 返回初始位置
            print("返回初始位置...")
            self.robot.move_ee(
                [0.2, -0.5, 1] + [0, np.pi / 2, np.pi / 2],
                'end',
                velocity=1.5
            )
            self._simulate_steps(200)

            # 闭合夹爪
            self.robot.close_gripper()
            self._simulate_steps(100)

            print("=== 抓取流程完成 ===\n")
            return True

        except Exception as e:
            print(f"抓取过程出错: {str(e)}")
            return False

    def _print_joint_states(self) -> None:
        """打印当前机器人关节状态"""
        joint_obs = self.robot.get_joint_obs()
        print("\n--- 当前关节状态 ---")

        # 打印机械臂关节
        for i, pos in enumerate(joint_obs['positions'][:self.robot.arm_num_dofs]):
            print(f"关节 {i + 1}: {pos:.4f} rad")

        # 打印夹爪状态
        if len(joint_obs['positions']) > self.robot.arm_num_dofs:
            gripper_pos = joint_obs['positions'][self.robot.arm_num_dofs]
            gripper_state = "闭合" if gripper_pos < 0.01 else "打开"
            print(f"夹爪: {gripper_state} ({gripper_pos:.4f} rad)")

        print("-------------------\n")

    def _simulate_steps(self, steps: int, delay: float = 1 / 240) -> None:
        """执行指定步数的仿真"""
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(delay)

    def _keyboard_listener(self) -> None:
        """键盘监听线程函数"""
        print("键盘监听线程已启动")
        print("操作指令:")
        print("  按回车键: 执行抓取")
        print("  输入 'r': 复位机器人")
        print("  输入 'n': 添加新物体")
        print("  输入 'q': 退出程序")

        while not self.quit_requested:
            try:
                user_input = input().strip().lower()

                if user_input == '':  # 回车键
                    self.grasp_requested = True
                    print("收到抓取指令...")
                elif user_input == 'r':
                    self.reset_requested = True
                    print("收到复位指令...")
                elif user_input == 'n':
                    self.add_objects_requested = True
                    print("收到添加物体指令...")
                elif user_input == 'q':
                    self.quit_requested = True
                    print("收到退出指令...")

            except (EOFError, KeyboardInterrupt):
                self.quit_requested = True
                break

    def run(self) -> None:
        """运行仿真主循环"""
        print("程序初始化完成，准备就绪！")

        # 添加初始物体
        self.add_objects(5)

        # 移动到初始位置
        self.robot.move_ee(
            [0.2, -0.5, 1] + [0, np.pi / 2, np.pi / 2],
            'end',
            velocity=0.8
        )

        # 等待物体稳定
        print("等待物体稳定...")
        self._simulate_steps(1500)
        print("物体已稳定，系统就绪！")

        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        keyboard_thread.start()

        # 主循环
        try:
            while not self.quit_requested:
                # 处理抓取请求
                if self.grasp_requested:
                    self.grasp_requested = False
                    success = self.perform_grasp()
                    if not success:
                        print("抓取失败，请重试")

                # 处理复位请求
                if self.reset_requested:
                    self.reset_requested = False
                    print("复位机器人...")
                    self.robot.reset()
                    self.robot.move_ee(
                        [0.2, -0.5, 1] + [0, np.pi / 2, np.pi / 2],
                        'end',
                        velocity=0.8
                    )
                    self._simulate_steps(200)

                # 处理添加物体请求
                if self.add_objects_requested:
                    self.add_objects_requested = False
                    self.add_objects(5)
                    print("等待新物体稳定...")
                    self._simulate_steps(500)

                # 继续仿真
                p.stepSimulation()
                time.sleep(1 / 240)

        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            print("清理资源并退出...")
            self.quit_requested = True
            p.disconnect()


if __name__ == "__main__":
    simulator = GraspingSimulator()
    simulator.run()