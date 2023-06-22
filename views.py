import open3d as o3d;
import numpy as np
import math
from flask import Blueprint, render_template, send_file, request, jsonify

views = Blueprint(__name__, "views")
filename = "models/ex.gltf"

# For changing the parameters use global variables for save the info
pcd = o3d.geometry.PointCloud()
points = []

voxelDownsampling = 0.1
method = 'ball'
point_neighbors = 30
point_radius = 0.1
poisson_depth = 5
# poisson_linear_fit = False
alpha = 0.3
min_radius = 0.05
max_radius = 1
poisson_scale = 1.1

# np_points = np.random.rand(100, 3)

# pointCloud = o3d.geometry.pointCloud()

def pointCloudMesh():
    global pcd
    global method
    global voxelDownsampling
    # global poisson_linear_fit
    global alpha
    global point_radius
    global point_neighbors
    global min_radius
    global max_radius
    global poisson_scale

    if(not np.any(np.asarray(pcd.points))):
        return
    downTmp = pcd.remove_duplicated_points()
    downTmp = downTmp.remove_statistical_outlier(nb_neighbors=30, std_ratio=1, print_progress=False)[0]
    down = downTmp.voxel_down_sample(voxel_size=voxelDownsampling)
    down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=point_radius, max_nn=point_neighbors))
    
    if(method == 'ball'):
        # distances = down.compute_nearest_neighbor_distance()
        # avg_dist = np.mean(distances)
        # radius = 3 * avg_dist
        # radius = [3*avg_dis, 2*3*avg_dist]
        radius = [min_radius, max_radius]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(down,o3d.utility.DoubleVector(radius))
    elif(method == 'poisson'):
        # down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(down, depth=poisson_depth, scale=poisson_scale)[0]
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(down, depth=poisson_depth)[0]
    elif(method == 'alpha'):
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(down, alpha)
    
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.85,0.24,0.28])
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh("models/ex.obj", mesh)
    # o3d.io.write_point_cloud("models/teste.xyz", down)
    o3d.io.write_triangle_mesh(filename, mesh)

@views.route("/")
def home():
    pointCloudMesh()
    # return render_template("index.html")
    return render_template("interface.html")

@views.route("/interface")
def interface():
    # pointCloudMesh()
    return render_template("interface.html")

@views.route("/model")
def profile():    
    global filename
    try:
        return send_file(filename, as_attachment=False)
    except Exception as e:
	    return str(e)

@views.route("/points", methods = ['POST'])
def get_points():
    global maxChunksPoints
    global maxPointsCounter
    global pcd
    global points
    # global np_points

    # pointTemp = []

    pcdTemp = o3d.geometry.PointCloud()

    
    args = request.json

    for p in args:
        print(p)
        x = float(p.get('x'))
        y = float(p.get('y'))
        z = float(p.get('z'))

        points.append([x, y, z])
        # print(points)

        maxPointsCounter = maxPointsCounter + 1
        np_points = np.array(points)
        # print(np_points)

    pcdTemp.points = o3d.utility.Vector3dVector(np_points)
    pcd = pcdTemp.remove_duplicated_points()
    pointCloudMesh()
    

    
    return args
    
@views.route("/parameters", methods = ['POST'])
def get_param():
    global method
    global alpha
    global point_neighbors
    global point_radius
    global poisson_depth
    global min_radius
    global max_radius
    global poisson_scale
  
    method = request.json.get("method")
    alpha = float(request.json.get("alpha"))
    point_neighbors = int(request.json.get("pointNeighbors"))
    point_radius = float(request.json.get("pointRadius"))
    poisson_depth = int(request.json.get("poissonDepth"))
    max_radius = float(request.json.get("maxRadius"))
    min_radius = float(request.json.get("minRadius"))
    poisson_scale = float(request.json.get("poissonScale"))
    
    pointCloudMesh()
    return request.json


@views.route("/downmodel")
def down_model():
    global filename
    global method
    pathFileObj = "models/geometry-" + method + ".obj"
    gltfMesh = o3d.io.read_triangle_mesh(filename)
    rotated_mesh = gltfMesh
    R = gltfMesh.get_rotation_matrix_from_xyz((-math.pi/2, 0, 0))
    rotated_mesh.rotate(R, center=(0,0,0))
    o3d.io.write_triangle_mesh(pathFileObj, rotated_mesh)
    try:
        return send_file(pathFileObj, as_attachment=True)
    except Exception as e:
	    return str(e)