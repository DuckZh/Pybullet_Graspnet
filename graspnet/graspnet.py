import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
import torch
from .grasp import GraspGroup

from .models.graspnet import GraspNet, pred_decode
from .models.collision_detector import ModelFreeCollisionDetector

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

class GraspBaseline:
    def __init__(self, checkpoint_path=os.path.join(ROOT_DIR, 'checkpoints/checkpoint-rs.tar'), num_point=20000, num_view=300, collision_thresh=0.01, voxel_size=0.01, device='cuda:0'):
        self.checkpoint_path = checkpoint_path
        self.num_point = num_point
        self.num_view = num_view
        self.collision_thresh = collision_thresh
        self.voxel_size = voxel_size
        self.device = device

        self.net = GraspNet(input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
        self.net.to(device)

        checkpoint = torch.load(checkpoint_path)
        self.net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        print("-> loaded checkpoint %s (epoch: %d)"%(checkpoint_path, start_epoch))
        self.net.eval()

    
    def preprocess_point_cloud(self, cloud):
        points = np.asarray(cloud.points)
        print('process points:', points.shape)
        N_points = points.shape[0]
        # sample points
        if N_points >= self.num_point:
            idxs = np.random.choice(N_points, self.num_point, replace=False)
        else:
            idxs1 = np.arange(N_points)
            idxs2 = np.random.choice(N_points, self.num_point-N_points, replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        points_sampled = points[idxs]

        end_points = dict()
        points_sampled = torch.from_numpy(points_sampled[np.newaxis].astype(np.float32))
        device = self.device
        points_sampled = points_sampled.to(device)
        end_points['point_clouds'] = points_sampled
        return end_points
    

    def get_grasps(self, cloud):
        end_points = self.preprocess_point_cloud(cloud)
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)
        return gg
    
    def collision_detection(self, gg, cloud):
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        gg = gg[~collision_mask]
        return gg
    
    
    def vis_grasps(self, gg, cloud):
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1, origin=[0, 0, 0])
        gg.nms()
        gg.sort_by_score()
        gg = gg[:50]
        grippers = gg.to_open3d_geometry_list()
        o3d.visualization.draw_geometries([cloud, *grippers, mesh_frame])

    def run(self, cloud, vis=True):
        gg = self.get_grasps(cloud)
        if self.collision_thresh > 0:
            gg = self.collision_detection(gg, np.asarray(cloud.points))
        
        if vis:
            self.vis_grasps(gg, cloud)

        return gg