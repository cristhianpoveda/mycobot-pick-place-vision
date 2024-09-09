#!/usr/bin/env python3

import struct
from copy import copy
import numpy as np
import math
import rospy
import rospkg
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose, Point
from pick_place_msgs.srv import DetectBottles, DetectBottlesResponse, VerifyPicking, VerifyPickingResponse

class ObjectDetector():

    def __init__(self):

        rospack = rospkg.RosPack()
        model_path = rospack.get_path('perception') + '/model/best.pt'

        self.ARM_TO_CAM_x = rospy.get_param("~arm_to_cam_x")
        self.ARM_TO_CAM_y = rospy.get_param("~arm_to_cam_y")
        self.ARM_TO_CAM_z = rospy.get_param("~arm_to_cam_z")
        self.ARM_TO_CAM_euler_z = np.radians(rospy.get_param("~arm_to_cam_euler_z"))
        self.BOTTLE_LENGHT = rospy.get_param("~bottle_lenght")
        self.PIXEL_SEARCH_LOWER = rospy.get_param("~pixel_search_lower")
        self.PIXEL_SEARCH_UPPER = rospy.get_param("~pixel_search_upper")
        self.MIN_B_T_DIST = rospy.get_param("~min_b_t_dist")
        
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        self.points = PointCloud2()

        self.image_np = np.zeros((640, 480, 3), dtype = "uint8")

        self.srv_detection = rospy.Service('~locate/bottle', DetectBottles, self.srv_detect)

        self.srv_verification = rospy.Service('~verify/picking', VerifyPicking, self.srv_verify)

        self.pub_image = rospy.Publisher('~detection', Image, queue_size=0)

        # self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)
        self.sub_image = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)

        # self.sub_points = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud, queue_size=1)
        self.sub_points = rospy.Subscriber('/camera/depth_registered/points/', PointCloud2, self.point_cloud, queue_size=1)

        rospy.loginfo("Initialized detection node")
    
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

        if math.isnan(cam_x) or math.isnan(cam_y) or math.isnan(cam_z):

            for i in range(self.PIXEL_SEARCH_LOWER, self.PIXEL_SEARCH_UPPER):

                new_x_idx = x_idx + i * self.points.point_step
                y_idx = new_x_idx + self.points.fields[1].offset
                z_idx = new_x_idx + self.points.fields[2].offset

                cam_x = self.bytearray_to_val(self.points.data[x_idx:(x_idx + 4)])
                cam_y = self.bytearray_to_val(self.points.data[y_idx:(y_idx + 4)])
                cam_z = self.bytearray_to_val(self.points.data[z_idx:(z_idx + 4)])

                if math.isnan(cam_x) or math.isnan(cam_y) or math.isnan(cam_z):
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
            pose.orientation.z = - pose_px_coord.z + self.ARM_TO_CAM_euler_z

        else:
            pose.position.x = -10
            pose.position.y = -10
            pose.position.z = -10

        return pose
    
    def get_cam_pose(self, keypoints):
        
        distances_centre = []
        centres = []
        poses = []
        z_coords = []
        scores = []
        valid = False
        centre_coord = Point()
        px_coord = Point()
        px_coord.x = -1
        px_coord.y = -1
        px_coord.z = -1 # euler angle z
        empty_pose = Pose()

        for b in keypoints:

            thread_x = int(b[0][0])
            thread_y = int(b[0][1])
            base_x = int(b[1][0])
            base_y = int(b[1][1])

            if not ((base_x == 0 and base_y == 0) or (thread_x == 0 and thread_y ==0)):

                lenght = math.sqrt((thread_x - base_x)**2 + (thread_y - base_y)**2)

                if lenght > self.MIN_B_T_DIST:

                    centre_coord.x = int(abs(thread_x + base_x) / 2)
                    centre_coord.y = int(abs(thread_y + base_y) / 2)
                    centre_coord.z = np.degrees(math.atan2((thread_y - base_y), (thread_x - base_x)))

                    pose = self.get_pose(centre_coord)

                    picking_distance = math.sqrt(pose.position.x**2 + pose.position.y**2)

                    if picking_distance < 0.241:

                        z = pose.position.z

                        if z > 0.025:

                            valid = True

                            centres.append(copy(centre_coord))

                            dist_centre = math.sqrt((centre_coord.x - 320)**2 + (centre_coord.y - 240)**2)

                            distances_centre.append(dist_centre)

                            poses.append(pose)

                            z_coords.append(z)

        if valid:

            max_dist = max(distances_centre)
            max_z = max(z_coords)

            for i, bottle_d in enumerate(distances_centre):

                score = 0.5 * (1 - (bottle_d / max_dist)) + 0.5 * (1 - (z_coords[i] / max_z))
                scores.append(score)

            selected_idx = scores.index(max(scores))
            
            return poses[selected_idx], centres[selected_idx]
        
        else:
            
            return empty_pose, px_coord
    
    def publish_image(self, pose_px_coord):

        cv2.circle(self.image_np, (pose_px_coord.x, pose_px_coord.y), 5, (0, 255, 0), -1)

        label = 'pixel: ' + str(pose_px_coord.x) + ', ' + str(pose_px_coord.y)
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.rectangle(self.image_np, (pose_px_coord.x, pose_px_coord.y+labelSize[1]+10), (pose_px_coord.x+labelSize[0], pose_px_coord.y-baseLine+10), (255, 255, 255), cv2.FILLED)
        cv2.putText(self.image_np, label, (pose_px_coord.x, pose_px_coord.y+labelSize[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(self.image_np, "rgb8"))

    def srv_detect(self, req=None):

        response = DetectBottlesResponse()

        pose_3d_coord = Pose()
        result = String()

        for i in range(3):

            detections = self.model(self.image_np, conf=0.7, verbose=False)
            keypoints = detections[0].keypoints.xy

            if len(keypoints[0]) != 0:

                pose_3d_coord, pos_px_coord = self.get_cam_pose(keypoints)

                if pos_px_coord.x == -1:
                    result.data = 'no accurate_detections'

                elif pose_3d_coord.position.x == -10:
                    result.data = 'no valid coord'

                elif not (-0.13 < pose_3d_coord.position.x < 0.13 and -0.27 < pose_3d_coord.position.y < -0.12):
                    result.data = 'outside box'

                else:
                    
                    result.data = 'success'

                    if self.pub_image.get_num_connections() > 0:

                        self.publish_image(pos_px_coord)

                    break  

            else:
                result.data = 'no detected bottles'

        response.pose = pose_3d_coord
        response.result.data = result.data

        return response
    
    def srv_verify(self, req=None):

        picked = False
        response = VerifyPickingResponse()

        for i in range(3):

            response.pick.data = False
            response.orientation.data = -1

            centre = Point()

            detections = self.model(self.image_np, conf=0.7, verbose=False)
            keypoints = detections[0].keypoints.xy

            if len(keypoints[0]) != 0:

                for point in keypoints:

                    thread_x = int(point[0][0])
                    thread_y = int(point[0][1])
                    base_x = int(point[1][0])
                    base_y = int(point[1][1])

                    if 190 < thread_x < 350 and 190 < base_x < 350 and 220 < thread_y < 300 and 220 < base_y < 300:
                        
                        centre.x = int(abs(thread_x + base_x) / 2)
                        centre.y = int(abs(thread_y + base_y) / 2)

                        pos = self.get_pose(centre)

                        if pos.position.z > 0.25:

                            if thread_x < base_x:
                                response.orientation.data = 180

                            else:
                                response.orientation.data = 0

                            response.pick.data = True
                            picked = True
                            break

                if picked: break

        return response
        
    
    def point_cloud(self, points_msg):
        
        self.points = points_msg

    def camera(self, image_msg):

        try:
            self.image_np = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
                
        except CvBridgeError as e:
            rospy.loginfo(f"Img msg to cvbride failed:\n{e}")

if __name__ == '__main__':

    rospy.init_node('detection_node')

    node = ObjectDetector()

    rospy.spin()