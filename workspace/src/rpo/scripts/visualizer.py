import open3d as o3d
import extended_octree_module
import random
import numpy as np

if __name__ == '__main__':

    file = '/home/appuser/data/models/infirmary_extended.ot'

    voxels = extended_octree_module.get_voxels(file)

    print(len(voxels))

    voxel_centers = []

    for v in voxels:
        voxel_centers.append([v["x"], v["y"], v["z"]])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    colors = np.random.rand(len(voxel_centers), 3)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    voxel_size = 0.05


    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    o3d.visualization.draw_geometries([vg])


