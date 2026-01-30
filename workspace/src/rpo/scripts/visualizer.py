import open3d as o3d
import extended_octree_module
import random
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

from matplotlib.colors import LinearSegmentedColormap

# export PYTHONPATH=/home/appuser/workspace/devel/lib:$PYTHONPATH

voxel_size = 0.05



def get_original_colors(voxels):
    return np.array([[v["r"], v["g"], v["b"]] for v in voxels])



def get_height_colors(voxel_centers):
    z_values = voxel_centers[:, 2]
    z_min, z_max = z_values.min(), z_values.max()
    z_normalized = (z_values - z_min) / (z_max - z_min)

    voxel_colors = np.stack([1.0 - z_normalized, 0.2 * np.ones_like(z_normalized), z_normalized], axis=1)

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



def get_dose_color(voxels):
    dose_values = np.array([v["dose"] for v in voxels])

    threshold = 280.0

    print(f"Elements above threshold: {np.sum(dose_values > 280.0)}")

    norm_doses = np.clip(dose_values / threshold, 0.0, 1.0)

    colors = plt.get_cmap("viridis")(norm_doses)[:, :3]

    # colors = []

    # for v in voxels:
    #     if v["dose"] >= threshold:
    #         colors.append([0.0, 1.0, 0.0])
    #     else:
    #         colors.append([1.0, 0.0, 0.0])

    # colors = np.array(colors)

    return colors



def create_voxel_edges(voxel_centers):

    cube_corners = np.array([
        [0, 0, 0], [1, 0, 0],
        [1, 1, 0], [0, 1, 0],
        [0, 0, 1], [1, 0, 1],
        [1, 1, 1], [0, 1, 1]
    ]) - 0.5

    edges = np.array([
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
    ])

    edge_color = [0, 0, 0]

    all_points = []
    all_lines = []
    all_colors = []

    for i, center in enumerate(voxel_centers):
        offset = i * 8
        corners = cube_corners * voxel_size + center
        all_points.append(corners)
        all_lines.append(edges + offset)
        all_colors.append([edge_color] * len(edges))

    points = np.vstack(all_points)
    lines = np.vstack(all_lines)
    colors = np.vstack(all_colors)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def get_lamp_positions(solution_file, grid_file):
    with open(grid_file, 'r') as f:
        lines = [line.strip().split() for line in f]

        data = [(int(kx), int(ky), int(kz), float(x), float(y), float(z)) for kx, ky, kz, x, y, z in lines]

        data.sort(key=lambda row: (row[0], row[1], row[2]))

    data = np.array(data)

    with open(solution_file, 'r') as f:
        times = np.array(list(map(float, f.readline().strip().split())))

    z_value = 0.525 # 2.425 # 0.525
    xy_positions = data[:, 3:5]
    positions = np.hstack([xy_positions, np.full((len(xy_positions), 1), z_value)])

    max_time = np.max(times)
    norm_times = times / max_time

    black_red = LinearSegmentedColormap.from_list("black_red", ["black", "red"])
    colors = black_red(norm_times)[:, :3]
    #colors = plt.get_cmap("Reds_r")(norm_times)[:, :3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vg_lamp = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=2*voxel_size)

    return vg_lamp



def get_lamp_positions2(pos_file):
    coords = []
    with open(pos_file, "r") as f:
        for line in f:
            if not line.strip():
                continue
            gx, gy, gz = map(float, line.split())
            coords.append([gx, gy, gz])
    coords = np.array(coords)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(coords)

    red = np.tile(np.array([[0.0, 0.0, 1.0]]), (len(coords), 1))
    pcd.colors = o3d.utility.Vector3dVector(red)

    vg_lamp = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 2*voxel_size)

    return vg_lamp




def load_path_voxels(path_file):
    coords = []
    with open(path_file, "r") as f:
        for line in f:
            if not line.strip():
                continue
            gx, gy, gz = map(float, line.split())
            coords.append([gx, gy, gz])
    return np.array(coords)



def create_path_voxels(path_xyz, voxel_size):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(path_xyz)

    red = np.tile(np.array([[1.0, 0.0, 0.0]]), (len(path_xyz), 1))
    pcd.colors = o3d.utility.Vector3dVector(red)

    vg_path = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size)

    return vg_path



def show_irradiance_maps(model_file, irradiance_dir):
    irradiance_files = sorted(glob.glob(os.path.join(irradiance_dir, '*.bin')))

    index = 0
    initial_voxels = extended_octree_module.get_irradiance_values(model_file, irradiance_files[index])

    voxel_centers = np.array([[v["x"], v["y"], v["z"]] for v in initial_voxels])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    voxel_colors = get_irradiance_color(initial_voxels)    
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.add_geometry(vg)

    def update_voxels(vis):
        nonlocal index
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




def show_dose(model_file, irradiance_dir, solution_file, grid_file):
    voxel_doses = extended_octree_module.get_dose_values(model_file, irradiance_dir, solution_file)

    voxel_centers = np.array([[v["x"], v["y"], v["z"]] for v in voxel_doses])

    voxel_edges = create_voxel_edges(voxel_centers)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    voxel_colors = get_dose_color(voxel_doses) 
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    vg_lamp = get_lamp_positions(solution_file, grid_file)

    o3d.visualization.draw_geometries([voxel_edges, vg, vg_lamp])


def show_path(model_file, irradiance_dir, path_file, pos_file):
    voxel_doses = extended_octree_module.get_dose_values(model_file, irradiance_dir, solution_file)

    voxel_centers = np.array([[v["x"], v["y"], v["z"]] for v in voxel_doses])

    voxel_edges = create_voxel_edges(voxel_centers)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    voxel_colors = get_dose_color(voxel_doses) 
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    path_xyz = load_path_voxels(path_file)

    vg_path = create_path_voxels(path_xyz, voxel_size)

    vg_lamp = get_lamp_positions2(pos_file)

    o3d.visualization.draw_geometries([voxel_edges, vg, vg_path, vg_lamp])



def show_objects(model_file):
    voxels = extended_octree_module.get_voxels(model_file)
    
    voxel_centers = np.array([[v["x"], v["y"], v["z"]] for v in voxels])
    
    voxel_edges = create_voxel_edges(voxel_centers)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)

    voxel_colors = get_height_colors(voxel_centers)
    pcd.colors = o3d.utility.Vector3dVector(voxel_colors)

    vg = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

    o3d.visualization.draw_geometries([voxel_edges, vg])



if __name__ == '__main__':

    model_file = '/home/appuser/data/models/infirmary_extended.ot'
    irradiance_dir = '/home/appuser/data/infirmary/infirmary_irradiance_xy'
    solution_file = '/home/appuser/data/infirmary2.sol'
    grid_file = '/home/appuser/data/grid.txt'
    # solution_file = '/home/appuser/data/ceiling.sol'
    # grid_file = '/home/appuser/data/ceiling_grid.txt'
    path_file = '/home/appuser/data/path.txt'
    pos_file = '/home/appuser/data/pos.txt'
    
    show_irradiance_maps(model_file, irradiance_dir)

    # show_dose(model_file, irradiance_dir, solution_file, grid_file)

    # show_objects(model_file)

    # show_path(model_file, irradiance_dir, path_file, pos_file)
