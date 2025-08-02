import random
random.seed(42)
from utilities import load_pcd
import numpy as np
from NDT import run_ndt
import open3d as o3d

ndt_poses = []  
transformation_matrix = np.eye(4)
sum_pcd = load_pcd(f"kitti_sample/00/000000.bin")
sum_pcd.paint_uniform_color([random.random(), random.random(), random.random()])
voxel_downsize = 0.9

for idx in range(0, 141):
    
    new_pcd = load_pcd(f"kitti_sample/00/{idx:06d}.bin")
    new_pcd = new_pcd.voxel_down_sample(voxel_downsize)
    
    transformed_new_pcd, transformation_matrix = run_ndt(new_pcd, sum_pcd, trans_init= transformation_matrix)
    
    transformed_new_pcd.paint_uniform_color([random.random(), random.random(), random.random()])
    sum_pcd = sum_pcd + transformed_new_pcd
    
    sum_pcd.voxel_down_sample(voxel_downsize)
    
    ndt_poses.append(transformation_matrix) 
    print(f"[{idx:03d}], t={transformation_matrix[:3,3]}")


ndt_poses = np.stack(ndt_poses, axis=0)   
np.save("ndt_poses.npy", ndt_poses)
o3d.io.write_point_cloud("ndt_pcd.ply", sum_pcd)
print("Saved NDT:", ndt_poses.shape, "poses for frames")