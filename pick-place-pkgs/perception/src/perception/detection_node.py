#!/usr/bin/env python3

import json
import struct
import numpy as np
import math
import rospy
import rospkg
import cv2
from tf.transformations import quaternion_from_euler
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, Point
from pick_place_srv.srv import DetectBottles, DetectBottlesResponse

class ObjectDetector():

    def __init__(self):

        rospack = rospkg.RosPack()
        rospack.get_path('detection')
        model_path = rospack.get_path('detection') + '/model/best.pt'
        config_file = rospack.get_path('detection') + '/config/default.json'

        with open(config_file, "r") as sim_params_file:
            params = json.load(sim_params_file)

            self.ARM_TO_CAM_x = params["arm_to_cam_x"]
            self.ARM_TO_CAM_y = params["arm_to_cam_y"]
            self.ARM_TO_CAM_z = params["arm_to_cam_z"]
            self.ARM_TO_CAM_euler_z = np.radians(params["arm_to_cam_euler_z"])
            self.BOTTLE_LENGHT = params["bottle_lenght"]
        
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        self.points = PointCloud2()

        self.image_np = np.zeros((640, 480, 3), dtype = "uint8")

        self.srv_detection = rospy.Service('/locate/bottle', DetectBottles, self.srv_detect)

        self.pub_image = rospy.Publisher('/detection', Image, queue_size=0)

        # self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)
        self.sub_image = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)

        # self.sub_points = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud, queue_size=1)
        self.sub_points = rospy.Subscriber('/camera/depth_registered/points/', PointCloud2, self.point_cloud, queue_size=1)

        rospy.loginfo("Initialized detection node")

    def get_cam_pose(self, keypoints):

        px_coord = Point()
        px_coord.x = -1
        px_coord.y = -1
        px_coord.z = -1 # euler angle z

        max_l = 0

        for b in keypoints:

            thread_x = int(b[0][0])
            thread_y = int(b[0][1])
            base_x = int(b[1][0])
            base_y = int(b[1][1])

            if not ((base_x == 0 and base_y == 0) or (thread_x == 0 and thread_y ==0)):

                lenght = np.sqrt(np.power(abs(thread_x - base_x),2) + np.power(abs(thread_y - base_y),2))

                if lenght > max_l:

                    max_l = lenght
                    px_coord.x = int(abs(thread_x + base_x) / 2)
                    px_coord.y = int(abs(thread_y + base_y) / 2)
                    px_coord.z = math.atan2((thread_y - base_y), (thread_x - base_x))
        
        return px_coord
    
    def bytearray_to_val(self, array):
        
        bytes = bytearray(array)
        float_tuple = struct.unpack('<f', bytes)
        val = float_tuple[0]
        return val

    def get_pose(self, pose_px_coord):

        pose = Pose()
        valid = False

        x_idx = self.points.row_step * pose_px_coord.y + self.points.point_step * pose_px_coord.x + self.points.fields[0].offset
        y_idx = x_idx + self.points.fields[1].offset
        z_idx = x_idx + self.points.fields[2].offset

        cam_x, cam_y, cam_z = 0, 0, 0

        cam_x = self.bytearray_to_val(self.points.data[x_idx:(x_idx + 4)])
        cam_y = self.bytearray_to_val(self.points.data[y_idx:(y_idx + 4)])
        cam_z = self.bytearray_to_val(self.points.data[z_idx:(z_idx + 4)])

        if math.isnan(cam_x) or math.isnan(cam_x) or math.isnan(cam_x):

            for i in range(-3, 4):

                new_x_idx = x_idx + i * self.points.point_step()
                y_idx = new_x_idx + self.points.fields[1].offset
                z_idx = new_x_idx + self.points.fields[2].offset

                cam_x = self.bytearray_to_val(self.points.data[x_idx:(x_idx + 4)])
                cam_y = self.bytearray_to_val(self.points.data[y_idx:(y_idx + 4)])
                cam_z = self.bytearray_to_val(self.points.data[z_idx:(z_idx + 4)])

                if math.isnan(cam_x) or math.isnan(cam_x) or math.isnan(cam_x):
                    continue

                else:
                    valid = True
                    break
        else:
            valid = True

        if valid:
            pose.position.x = cam_x - self.ARM_TO_CAM_x
            pose.position.y = - cam_y - self.ARM_TO_CAM_y
            pose.position.z = self.ARM_TO_CAM_z - cam_z

            euler_z_world = - pose_px_coord.z + self.ARM_TO_CAM_euler_z
            orientation = quaternion_from_euler(0, 0, euler_z_world)
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]

        else:
            pose.position.x = -10
            pose.position.y = -10
            pose.position.z = -10

        return pose
    
    def publish_image(self, pose_px_coord):

        cv2.circle(self.image_np, (pose_px_coord.x, pose_px_coord.y), 5, (0, 255, 0), -1)

        label = 'pixel: ' + str(pose_px_coord.x) + ', ' + str(pose_px_coord.y)
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.rectangle(self.image_np, (pose_px_coord.x, pose_px_coord.y+labelSize[1]+10), (pose_px_coord.x+labelSize[0], pose_px_coord.y-baseLine+10), (255, 255, 255), cv2.FILLED)
        cv2.putText(self.image_np, label, (pose_px_coord.x, pose_px_coord.y+labelSize[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(self.image_np, "rgb8"))

    def srv_detect(self, req=None):

        pose_px_coord = Point()
        pose_3d_coord = Pose()
        result = String()

        detections = self.model(self.image_np, conf=0.7, verbose=False)
        keypoints = detections[0].keypoints.xy

        if len(keypoints[0]) != 0:

            pose_px_coord = self.get_cam_pose(keypoints)

            if pose_px_coord.x != -1:

                pose_3d_coord = self.get_pose(pose_px_coord)

                if pose_3d_coord.position.x == -10:
                    result.data = 'no valid coord'

                else: result.data = 'success'

                if self.pub_image.get_num_connections() > 0:

                    self.publish_image(pose_px_coord)      

            else:
                result.data = 'no accurate detections'

        else:
            result.data = 'no detected bottles'

        print(pose_3d_coord)

        response = DetectBottlesResponse()
        response.pose = pose_3d_coord
        response.result.data = result.data

        return response
    
    def point_cloud(self, points_msg):
        
        self.points = points_msg

    def camera(self, image_msg):

        try:
            self.image_np = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
                
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':

    rospy.init_node('detection_node')

    node = ObjectDetector()

    rospy.spin()