#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from collada import Collada
from mesh_msgs.msg import MeshGeometry
from mesh_msgs.msg import MeshTriangleIndices
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray

import os
BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
print(BASE_DIR)

def talker():
    # Create the publisher
    pub = rospy.Publisher('mesh', PolygonArray, queue_size=10)

    # Initialize the node
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Get mesh data from collada file
    # mesh = Collada('../surfaces/L_ELBOW_Y_LINK.dae')
    mesh = Collada(BASE_DIR + '/surfaces/mesh2.dae')

    # Get the point cloud from the mesh
    geom = mesh.geometries[0]           # geometry
    triset = geom.primitives[0]         # triangle set
    triangles = triset.vertex_index     # list of triangles
    vertices = triset.vertex            # list of vertices
    normals = triset.normal             # list of normals

    # Get the number of triangles and normals
    n = len(triset)

    # Get the number of vertices
    m = len(vertices)

    # # Create the mesh geometry message
    # mesh_msg = MeshGeometry()
    # mesh_msg.faces = []             # MeshTriangleIndices[]
    # mesh_msg.vertices = []          # Point[]
    # mesh_msg.vertex_normals = []    # Point[]

    # # Fill the mesh geometry message
    # for i in range(n):
    #     normal = Point()
    #     normal.x = normals[i][0]
    #     normal.y = normals[i][1]
    #     normal.z = normals[i][2]
    #     mesh_msg.vertex_normals.append(normal)
    #     triangle = MeshTriangleIndices()
    #     triangle.vertex_indices = [
    #         triset.vertex_index[i][0],
    #         triset.vertex_index[i][1],
    #         triset.vertex_index[i][2]
    #     ]
    #     mesh_msg.faces.app
    # end(triangle)
    # for i in range(m):
    #     vertex = Point()
    #     vertex.x = vertices[i][0]
    #     vertex.y = vertices[i][1]
    #     vertex.z = vertices[i][2]
    #     mesh_msg.vertices.append(vertex)

    # Create the PolygonArray message
    polygon_msg = PolygonArray()
    polygon_msg.header.frame_id = 'my_frame'
    polygon_msg.header.stamp = rospy.Time.now()
    polygon_msg.polygons = []       # PolygonStamped[]
    polygon_msg.labels = []         # uint32[]
    polygon_msg.likelihood = []     # string[]
    for i in range(n):
        polygon = PolygonStamped()
        polygon.header.frame_id = 'my_frame'
        polygon.header.stamp = rospy.Time.now()
        pointA = Point32()
        pointA.x = triset.vertex[triset.vertex_index[i][0]][0]
        pointA.y = triset.vertex[triset.vertex_index[i][0]][1]
        pointA.z = triset.vertex[triset.vertex_index[i][0]][2]
        pointB = Point32()
        pointB.x = triset.vertex[triset.vertex_index[i][1]][0]
        pointB.y = triset.vertex[triset.vertex_index[i][1]][1]
        pointB.z = triset.vertex[triset.vertex_index[i][1]][2]
        pointC = Point32()
        pointC.x = triset.vertex[triset.vertex_index[i][2]][0]
        pointC.y = triset.vertex[triset.vertex_index[i][2]][1]
        pointC.z = triset.vertex[triset.vertex_index[i][2]][2]
        polygon.polygon.points = [pointA, pointB, pointC]
        polygon_msg.polygons.append(polygon)
        polygon_msg.labels.append(i)
    polygon_msg.likelihood = [0 for i in range(n)]

    # Generate random MONO8 cv2 matrix of size 6*22.
    # The matrix should be generated randomly.
    # img_numpy = np.random.randint(0, 255, (6, 22), dtype=np.uint8)
    # # img_numpy = np.zeros((6, 22), dtype=np.uint8)
    # for i in range(6):
    #     for j in range(22):
    #         img_numpy[i][j] = 10
    # img_numpy_1D = img_numpy.ravel()

    # Change polygon color according to the value of the pixel
    # for i in range(n):
    #     if i < len(img_numpy_1D):
    #         polygon_msg.likelihood.append(img_numpy_1D[i])
    #     else:
    #         polygon_msg.likelihood.append(0)


    # Publish the message in ROS loop
    while not rospy.is_shutdown():
        img_numpy = np.random.randint(0, 255, (6, 22), dtype=np.uint8)
        # img_numpy = np.zeros((6, 22), dtype=np.uint8)
        # for i in range(6):
        #     for j in range(22):
        #         img_numpy[i][j] = 10
        img_numpy_1D = img_numpy.ravel()

        # Change polygon color according to the value of the pixel
        for i in range(len(img_numpy_1D)):
            polygon_msg.likelihood[i] = img_numpy_1D[i]
            polygon_msg.likelihood[i+n/2] = img_numpy_1D[i]
        pub.publish(polygon_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
