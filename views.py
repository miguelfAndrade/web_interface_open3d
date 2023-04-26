import open3d as o3d;
import numpy as np
from flask import Blueprint, render_template, send_file, request, jsonify

views = Blueprint(__name__, "views")
filename = "models/ex.gltf"

# For changing the parameters use global variables for save the info

maxChunksPoints = 100
maxPointsCounter = 0
pcd = o3d.geometry.PointCloud()
points = []

voxelDownsampling = 0.1
method = 'poisson'
point_neighbors = 30
point_radius = 0.1
poisson_depth = 5
poisson_width = 0
poisson_scale = 1.1
poisson_linear_fit = False

# np_points = np.random.rand(100, 3)

# pointCloud = o3d.geometry.pointCloud()

def pointCloudMesh():
    global pcd
    global method
    if(not np.any(np.asarray(pcd.points))):
        return
    downTmp = pcd.remove_duplicated_points()
    down = downTmp.voxel_down_sample(voxel_size=0.1)
    print(method)
    if(method == 'ball'):
        print('Entrou no ball')
        distances = down.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = 3 * avg_dist
        down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(down,o3d.utility.DoubleVector([radius, radius * 2]))
    elif(method == 'poisson'):
        print('Entrou no poisson')
        down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(down, depth=10, width=0, scale=1.1, linear_fit=False)[0]
    
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0,0,1])
    # o3d.visualization.draw_geometries([mesh])
    # o3d.io.write_triangle_mesh("models/ex.obj", mesh)
    o3d.io.write_triangle_mesh(filename, mesh)

@views.route("/")
def home():
    pointCloudMesh()
    return render_template("index.html")

@views.route("/interface")
def interface():
    # pointCloudMesh()
    return render_template("interface.html")

@views.route("/model")
def profile():    
    try:
        return send_file(filename, as_attachment=False)
    except Exception as e:
	    return str(e)

@views.route("/points", methods = ['POST', 'GET'])
def get_points():
    global maxChunksPoints
    global maxPointsCounter
    global pcd
    global points
    # global np_points

    # pointTemp = []

    pcdTemp = o3d.geometry.PointCloud()

    args = request.args
    x = float(args.get('x'))
    y = float(args.get('y'))
    z = float(args.get('z'))

    points.append([x, y, z])
    # print(points)

    maxPointsCounter = maxPointsCounter + 1
    np_points = np.array(points)
    # print(np_points)

    
    if(maxPointsCounter > maxChunksPoints):
        pcdTemp.points = o3d.utility.Vector3dVector(np_points)
        pcdClean = pcdTemp.remove_non_finite_points(True, True)
        pcd = pcdClean.remove_duplicated_points()
        # print(np.asarray(pcdClean.points))
        pointCloudMesh()
        maxPointsCounter = 0
    
    return args
    
@views.route("/parameters", methods = ['POST', 'GET'])
def get_param():
    # print(request.json.get("method"))
    global method
    method = str(request.json.get("method"))
    pointCloudMesh()
    return request.json