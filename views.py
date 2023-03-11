import open3d as o3d;
import numpy as np
from flask import Blueprint, render_template, send_file, request, jsonify

views = Blueprint(__name__, "views")
filename = "models/ex.gltf"

# pointCloud = o3d.geometry.pointCloud()

def pointCloudMesh():
    pcd = o3d.io.read_point_cloud("data/sensor_data.xyz")
    # downTmp = pcd.voxel_down_sample(voxel_size=0.1)
    down = pcd.remove_duplicated_points()
    distances = down.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(down,o3d.utility.DoubleVector([radius, radius * 2]))
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(down, depth=10, width=0, scale=1.1, linear_fit=False)[0]
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0,0,1])
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh("models/ex.obj", mesh)
    o3d.io.write_triangle_mesh("models/ex.gltf", mesh)

@views.route("/")
def home():
    # pointCloudMesh()
    return render_template("index.html")

@views.route("/interface")
def interface():
    pointCloudMesh()
    return render_template("interface.html")

@views.route("/model")
def profile():    
    try:
        return send_file(filename, as_attachment=False)
    except Exception as e:
	    return str(e)

@views.route("/points", methods = ['POST'])
def get_points():
    points = request.args
    return jsonify(points)
    
