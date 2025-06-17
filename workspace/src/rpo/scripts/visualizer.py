import open3d as o3d
import extended_octree_module
import random
import numpy as np

# export PYTHONPATH=/home/appuser/workspace/devel/lib:$PYTHONPATH

if __name__ == '__main__':

    file = '/home/appuser/data/models/infirmary_extended.ot'

    voxels = extended_octree_module.get_voxels(file)

    print(len(voxels))

    voxel_centers = []
    voxel_colors = []

    for v in voxels:
        voxel_centers.append([v["x"], v["y"], v["z"]])
        voxel_colors.append([v["r"], v["g"], v["b"]])

    voxel_centers = np.array(voxel_centers)
    voxel_colors = np.array(voxel_colors)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    # z_values = voxel_centers[:, 2]
    # z_min, z_max = z_values.min(), z_values.max()
    # z_normalized = (z_values - z_min) / (z_max - z_min)

    # colors = np.stack([1.0 - z_normalized, 0.2 * np.ones_like(z_normalized), z_normalized], axis=1)
    #colors = np.random.rand(len(voxel_centers), 3)
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    voxel_size = 0.05


    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    o3d.visualization.draw_geometries([vg])


