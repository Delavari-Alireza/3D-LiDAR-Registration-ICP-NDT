import random
import open3d as o3d
import copy
import numpy as np
from point_cloud_registration import NDT

ndt = NDT()

def run_ndt(source_org, target_org,
                    trans_init=np.eye(4) ):

    source = copy.deepcopy(source_org)
    target = copy.deepcopy(target_org)

    source_pts = np.asarray(source.points)
    target_pts = np.asarray(target.points)

    ndt.set_target(target_pts)
    transformation_matrix = ndt.align(source_pts , trans_init ) 

    
    aligned = copy.deepcopy(source_org)
    aligned.transform(transformation_matrix)
    return aligned, transformation_matrix