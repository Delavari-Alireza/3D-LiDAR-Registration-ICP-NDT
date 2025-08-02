import numpy as np
import open3d as o3d
import os
from PIL import Image
import matplotlib.pyplot as plt
import copy
import random
random.seed(42)

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      )
counter = 0
def save_and_show_rendered_image(pcd, fileName="shot"):
    global counter 
    counter += 1
    output_path = f"{fileName}{counter}.png"
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(output_path)
    vis.destroy_window()
    print(f"Saved to {output_path}")
    img = Image.open(output_path)
    plt.imshow(img)
    plt.axis('off')
    plt.show()

def load_pcd(path, voxel_size=None):
    pcd = o3d.geometry.PointCloud(
        o3d.utility.Vector3dVector(
            np.fromfile(path, dtype=np.float32).reshape(-1,4)[:,:3]
        )
    )
    if voxel_size:
        pcd = pcd.voxel_down_sample(voxel_size)
    return pcd


def plot_pcd(pcds, fileName="tmp" ,  show=True ):
    if not isinstance(pcds, list):
        pcds = [pcds]


    output_path = f"{fileName}.png"
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for pcd in pcds:
        vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(output_path)
    vis.destroy_window()

    # print(f"Saved to {output_path}")
    img = Image.open(output_path)
    img_return = img.copy()
    if show:
        plt.imshow(img)
        plt.axis('off')
        plt.show()
    return img_return

def plot_pcds(pcds, fileName="tmp" ,  show=True , intractive = False):
    if not isinstance(pcds, list):
        pcds = [pcds]



    # Assign unique colors from a palette
    palette = [
        [1, 0.706, 0],    # Red
        [0, 0.651, 0.929],    # Green
        [0, 0, 1],    # Blue
        [1, 1, 0],    # Yellow
        [1, 0, 1],    # Magenta
        [0, 1, 1],    # Cyan
        [0.5, 0.5, 0.5],  # Gray
    ]


    colored_pcds = []
    for i, pcd in enumerate(pcds):
        rgb = palette[i % len(palette)]
        
        colored_pcds.append(pcd.paint_uniform_color(rgb))

    if intractive:
        o3d.visualization.draw_geometries(colored_pcds)

    # Offscreen render
    output_path = f"{fileName}.png"
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for pcd in colored_pcds:
        vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(output_path)
    vis.destroy_window()

    # print(f"Saved to {output_path}")
    img = Image.open(output_path)
    img_return = img.copy()
    if show:
        plt.imshow(img)
        plt.axis('off')
        plt.show()
    return img_return



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

def run_icp_plot(source_org , target_org , threshold = 1.0 , trans_init = np.eye(4) , max_iteration=3000):
    

    source = copy.deepcopy(source_org)
    target = copy.deepcopy(target_org)

    reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    source_temp = copy.deepcopy(source)
    transformed_pcd = source_temp.transform(reg_p2p.transformation)

    img_before = plot_pcd([source,target] , show=False)
    img_after  = plot_pcd([transformed_pcd,target] ,show=False)
    # img_before = plot_pcd([source,target])
    # img_after  = plot_pcd([transformed_pcd,target] )    
    fig, axes = plt.subplots(1, 2, figsize=(24, 12))
    axes[0].imshow(img_before)
    axes[0].set_title("Before ICP")
    axes[0].axis('off')
    axes[1].imshow(img_after)
    axes[1].set_title("After ICP")
    axes[1].axis('off')
    plt.show()
    return reg_p2p