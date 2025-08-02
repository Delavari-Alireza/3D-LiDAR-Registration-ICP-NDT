import open3d as o3d
import copy
import numpy as np

def run_icp(source_org, target_org,
                    threshold=1.0, trans_init=np.eye(4),
                    max_iteration=3000 ):

    source = copy.deepcopy(source_org)
    target = copy.deepcopy(target_org)

    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iteration
        ))

    
    aligned = copy.deepcopy(source_org)
    aligned.transform(reg_p2p.transformation)
    return aligned, reg_p2p.transformation.copy() , reg_p2p.fitness, reg_p2p.inlier_rmse