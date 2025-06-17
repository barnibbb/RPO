import open3d as o3d
import extended_octree_module
import random
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

# export PYTHONPATH=/home/appuser/workspace/devel/lib:$PYTHONPATH

voxel_size = 0.05



def get_original_colors(voxels):
    return np.array([[v["r"], v["g"], v["b"]] for v in voxels])



def get_height_colors(voxel_centers):
    voxel_colors = []

    z_values = voxel_centers[:, 2]
    z_min, z_max = z_values.min(), z_values.max()
    z_normalized = (z_values - z_min) / (z_max - z_min)

    colors = np.stack([1.0 - z_normalized, 0.2 * np.ones_like(z_normalized), z_normalized], axis=1)

    return np.array(voxel_colors)



def get_random_colors(voxel_centers):
    voxel_colors = np.random.rand(len(voxel_centers), 3)

    return np.array(voxel_colors)



def get_type_color(voxels):
    voxel_colors = []

    for v in voxels:
        if v["type"] == 1:
            voxel_colors.append([255.0 / 255.0, 120.0 / 255.0, 0.0 / 255.0])
        elif v["type"] == 2:
            voxel_colors.append([254.0 / 255.0, 47.0 / 255.0, 125.0 / 255.0])
        elif v["type"] == 4:
            voxel_colors.append([255.0 / 255.0, 120.0 / 255.0, 0.0 / 255.0])
        elif v["type"] == 5:
            voxel_colors.append([15.0 / 255.0, 82.0 / 255.0, 186.0 / 255.0])
        else:
            voxel_colors.append([0.0, 0.0, 0.0])

    return np.array(voxel_colors)



def get_irradiance_color(voxels):
    voxel_values = np.array([v["value"] for v in voxels])

    clip_value = np.percentile(voxel_values, 99)
    voxel_values_clipped = np.clip(voxel_values, 0, clip_value)

    value_min = voxel_values_clipped.min()
    value_max = voxel_values_clipped.max()
    value_normalized = (voxel_values_clipped - value_min) / (value_max - value_min)

    colormap = plt.get_cmap("viridis")

    voxel_colors = colormap(value_normalized)[:, :3]

    # voxel_colors = np.stack([value_normalized, 0.0 * value_normalized, value_normalized], axis=1)

    return voxel_colors



if __name__ == '__main__':
    
    model_file = '/home/appuser/data/models/infirmary_extended.ot'
    # irradiance_file = '/home/appuser/data/irradiance_infirmary/32751_32760.bin'

    irradiance_dir = '/home/appuser/data/irradiance_infirmary'
    irradiance_files = sorted(glob.glob(os.path.join(irradiance_dir, '*.bin')))
    index = 0

    initial_voxels = extended_octree_module.get_irradiance_values(model_file, irradiance_files[index])
    voxel_centers = np.array([[v["x"], v["y"], v["z"]] for v in initial_voxels])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    voxel_colors = get_irradiance_color(initial_voxels)    
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    # o3d.visualization.draw_geometries([vg])

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.add_geometry(vg)

    def update_voxels(vis):
        global index
        index = (index + 1) % len(irradiance_files)
        print(f"Loading: {irradiance_files[index]}")
        voxels = extended_octree_module.get_irradiance_values(model_file, irradiance_files[index])
        colors = get_irradiance_color(voxels)  
        pcd.colors = o3d.utility.Vector3dVector(colors)

        new_vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
        vis.clear_geometries()
        vis.add_geometry(new_vg)
        vis.poll_events()
        vis.update_renderer()

    RIGHT_ARROW = 262
    vis.register_key_callback(RIGHT_ARROW, update_voxels)

    print("Press → to step through irradiance frames.")
    vis.run()
    vis.destroy_window()


