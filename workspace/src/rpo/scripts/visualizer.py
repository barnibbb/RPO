import open3d as o3d
import extended_octree_module

if __name__ == '__main__':

    file = '/home/appuser/data/models/infirmary_extended.ot'

    voxels = extended_octree_module.get_voxels(file)


    cubes = []
    cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    cube.translate([0.5, 0.5, 0.5])
    cube.paint_uniform_color([0.3, 0, 0.3])
    cubes.append(cube)
    cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    cube.translate([1.5, 0.5, 0.5])
    cube.paint_uniform_color([0.7, 0, 0.7])
    cubes.append(cube)

    o3d.visualization.draw_geometries(cubes)



