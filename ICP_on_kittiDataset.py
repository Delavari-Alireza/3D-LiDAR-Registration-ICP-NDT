import random
random.seed(42)
from utilities import load_pcd
import numpy as np
from ICP import run_icp
import open3d as o3d

icp_poses = []  

transformation_matrix = np.eye(4)
sum_pcd = load_pcd(f"kitti_sample/00/000000.bin")
sum_pcd.paint_uniform_color([random.random(), random.random(), random.random()])
voxel_downsize = 0.9

for idx in range(0, 141):
    
    new_pcd = load_pcd(f"kitti_sample/00/{idx:06d}.bin")
    new_pcd = new_pcd.voxel_down_sample(voxel_downsize)
    
    transformed_new_pcd, transformation_matrix , fit, rmse = run_icp(new_pcd, sum_pcd, trans_init= transformation_matrix , threshold=2.0, max_iteration=300)
    transformed_new_pcd.paint_uniform_color([random.random(), random.random(), random.random()])
    sum_pcd = sum_pcd + transformed_new_pcd
    
    sum_pcd.voxel_down_sample(voxel_downsize)
    
    icp_poses.append(transformation_matrix) 
    print(f"[{idx:03d}] fit={fit:.3f}, rmse={rmse:.3f}, t={transformation_matrix[:3,3]}")


icp_poses = np.stack(icp_poses, axis=0)   
np.save("icp_poses.npy", icp_poses)
o3d.io.write_point_cloud("icp_pcd.ply", sum_pcd)
print("Saved ICP:", icp_poses.shape, "poses for frames")