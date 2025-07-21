# Pybullet_Graspnet
本项目结合 Pybullet 与 GraspNet 实现机械臂6D夹爪位姿估计，支持键盘控制实现环境中多个物体的连续抓取。夹爪位姿估计基于 GraspNet 官方代码（https://github.com/graspnet/graspnet-baseline ），仿真环境为 Pybullet，测试环境为 Win11 + GeForce RTX 3060 + CUDA 11.8。

<div align="center">    
    <img src="https://github.com/chenxi-wang/materials/blob/master/graspnet-baseline/doc/gifs/scene_0114.gif", width="240", alt="scene_0114" />
    <img src="https://github.com/chenxi-wang/materials/blob/master/graspnet-baseline/doc/gifs/scene_0116.gif", width="240", alt="scene_0116" />
    <img src="https://github.com/chenxi-wang/materials/blob/master/graspnet-baseline/doc/gifs/scene_0117.gif", width="240", alt="scene_0117" />
    <br> Top 50 grasps detected by grasp baseline model.
    <img width="730" height="480" alt="demo2" src="https://github.com/user-attachments/assets/291d59cb-e544-49b8-939e-6f253028a17a" />
    <br> Pybullet + Grasp demo video
</div>

## Requirements
- torch==1.12.0
- pybullet
- open3d
- numpy==1.23.4
- opencv-python
- grasp_nms


## Installation
1. 安装 KNN 和 PointNet2
```
cd libs/knn
python setup.py install
cd libs/pointnet2
python setup.py install
```
2. 安装库
```
pip install -r requirements.txt
```

## Run Demo
在项目根目录下运行：
```
python pybullet_grasp.py
```
可以通过键盘进行场景和机械臂的控制：
```
操作指令:
  按回车键: 执行抓取
  输入 'r': 复位机器人
  输入 'n': 添加新物体
  输入 'q': 退出程序
```
## Result
演示效果可参考B站视频：
https://www.bilibili.com/video/BV1jqNnzjEPj/?share_source=copy_web&vd_source=c9b7a846ff7e069af4893d38f288c751
<img width="1751" height="1081" alt="screenshot_2025-07-21_12-55-32" src="https://github.com/user-attachments/assets/fefddb5d-03bd-4d81-8d55-44c43fe366ce" />
<img width="2140" height="1144" alt="screenshot_2025-07-21_12-57-52" src="https://github.com/user-attachments/assets/90f94502-9f12-4cf6-b744-3013f4db0b1d" />
<img width="2143" height="1088" alt="screenshot_2025-07-21_12-58-25" src="https://github.com/user-attachments/assets/176680c0-c3c5-4bf0-b635-5230c2f6e99e" />


