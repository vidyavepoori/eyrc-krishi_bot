#! /usr/bin/env python3


# Team ID:		    [ KB_1492 ]
# Author List:		[ vidya vepoori , ch pranathi ]
# Filename:			percepStack.py
# Functions:		callback(),image_processing(),depth_processing()

##################### IMPORT MODULES #######################

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
import numpy as np
import message_filters
import pyrealsense2
import tf
import tf2_ros


###########################################################


############################################################


def ImageProcessing(img):

    r_center = []
    y_center = []

    # cvt img to hsv
    hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # cv2.imshow("img",img)

    # thresholding
    # for yellow bell pepper

    y_lower = np.array([92, 172, 0])
    y_upper = np.array([114, 255, 255])

    # for red bell pepper

    r_lower = np.array([110, 61, 0])
    r_upper = np.array([129, 255, 255])

    # masking for only yellow
    y_mask = cv2.inRange(hsv_img, y_lower, y_upper)

    # masking for only red
    r_mask = cv2.inRange(hsv_img, r_lower, r_upper)

    # object segmenattion for only red
    r_output = cv2.bitwise_and(img, img, mask=r_mask)

    # object segmetationn fro ony yellow
    y_output = cv2.bitwise_and(img, img, mask=y_mask)

    # contour for yellow object
    y_countor, y_h = cv2.findContours(
        y_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # contour for red object
    r_countor, r_h = cv2.findContours(
        r_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # draw contour:
    # finding center for yellow object
    for c in y_countor:

        y_area = cv2.contourArea(c)

        if y_area >= 500:
            cv2.drawContours(img, [c], 0, (255, 0, 0), 2)
            # cv2.imshow("y_countour",img)

            # centers
            y_m = cv2.moments(c)

            y_cx = int(y_m["m10"]/y_m["m00"])  # x coord
            y_cy = int(y_m["m01"]/y_m["m00"])  # y coord

            # append centers to a list
            y_center.append(y_cx)
            y_center.append(y_cy)

    # finding center fro red object
    for c in r_countor:

        r_area = cv2.contourArea(c)

        if r_area >= 500:
            cv2.drawContours(img, [c], 0, (255, 0, 0), 2)
            # cv2.imshow("r_countour",img)

            # centers
            r_m = cv2.moments(c)

            r_cx = int(r_m["m10"]/r_m["m00"])  # x coord
            r_cy = int(r_m["m01"]/r_m["m00"])  # y coord

            # append centers to a list
            r_center.append(r_cx)
            r_center.append(r_cy)

    return y_center, r_center

def Depth_Processing(img, y_center, r_center):

    y_depth_val = []
    r_depth_val = []

    if len(y_center) >=1:
        
        y_x = y_center[0]
        y_y = y_center[1]
        y_depth = img[y_y, y_x]  # encoding is 32FC1 so it in meters

        y_depth_val.append(y_depth)


    if len(r_center) >=1:
    
        r_x = r_center[0]
        r_y = r_center[1]
        r_depth = img[r_y, r_x]
        r_depth_val.append(r_depth)

    

    return y_depth_val, r_depth_val


def convert_depth_to_phys_coord_using_realsense(y_center, r_center, y_depth_val, r_depth_val, cameraInfo):

    y_coord = []
    r_coord = []

    if len(y_center) >= 1:

        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.K[2]
        _intrinsics.ppy = cameraInfo.K[5]
        _intrinsics.fx = cameraInfo.K[0]
        _intrinsics.fy = cameraInfo.K[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.D]
        
        y_x = y_center[0]
        y_y = y_center[1]
        y_z = y_depth_val[0]
        y_result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics,[y_x, y_y], y_z)
        y_coord.append(y_result)

    if len(r_center) >= 1:

        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = cameraInfo.width
        _intrinsics.height = cameraInfo.height
        _intrinsics.ppx = cameraInfo.K[2]
        _intrinsics.ppy = cameraInfo.K[5]
        _intrinsics.fx = cameraInfo.K[0]
        _intrinsics.fy = cameraInfo.K[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.D]
        
        r_x = r_center[0]
        r_y = r_center[1]
        r_z = r_depth_val[0]
        r_result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics,[r_x, r_y], r_z)
        r_coord.append(r_result)

    return y_coord, r_coord


# intinilizing list for storingvales of centers and depth dist

def callback(rgb, depth, camera_info):

    # publisher topic to publish pose of yellow and red brll epper wrt to ebot_base on resp. topics

    pub_y = rospy.Publisher("/yellow_pose", Pose, queue_size=10)
    pub_r = rospy.Publisher("/red_pose", Pose, queue_size=10)
    pub_y_pose = Pose()
    pub_r_pose = Pose()
    
    # object for cvbridge 

    br = CvBridge()

    # for transforms

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    ############################################################################

    # color img cvt ros img into cv2 format 

    rgb_image = br.imgmsg_to_cv2(rgb, 'bgr8')
    # cv2.imshow("rgb_image", rgb_image)
    y_center, r_center = ImageProcessing(rgb_image)
    # print("y_center: ", y_center, "\nr_center: ", r_center)

    # depth image

    depth_image = br.imgmsg_to_cv2(depth, "passthrough")
    y_depth_val, r_depth_val = Depth_Processing(
        depth_image, y_center, r_center)

    # print("y_depth: ", y_depth_val, "\nr_depth: ", r_depth_val)  

    y_coord, r_coord = convert_depth_to_phys_coord_using_realsense(
        y_center, r_center, y_depth_val, r_depth_val, camera_info)

    # print("y_coord: ",y_coord,"\nr_coord :",r_coord) 

    ############################################################################
      
    # publish 1 if fruit detected and able to reach else publish 0

    pub_status = rospy.Publisher("/task_info", Float64, queue_size= 10)
    _status = Float64()
    _status.data = 0
    pub_status.publish(_status)

    
    # transformer
    # tf_broadcaster   

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    try:

        # sending fruit pose wrt to camera

        if len(y_center) >= 1:

            br.sendTransform((y_coord[0][0], y_coord[0][1], y_coord[0][2]),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            'yellow', 'camera_rgb_frame2',
                            )

            rate.sleep()          

        if len(r_center) >= 1:

            br.sendTransform((r_coord[0][0], r_coord[0][1], r_coord[0][2]),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            'red', 'camera_rgb_frame2',)

            rate.sleep()

#///////////////////////////////////////////////////////////////////////////////////////////////////#      
              
        # getiing fruit pose wrt to ebot base if present broadcasting it to ebot base

        if len(y_center) >= 1:

            y_pose = tfBuffer.lookup_transform(
                "ebot_base", "yellow", rospy.Time(0))

            y_x = y_pose.transform.translation.x
            y_y = y_pose.transform.translation.y
            y_z = y_pose.transform.translation.z
            y_roll = y_pose.transform.rotation.x
            y_pitch = y_pose.transform.rotation.y
            y_yaw = y_pose.transform.rotation.z
            y_w = y_pose.transform.rotation.w

            pub_y_pose.position.x = y_x
            pub_y_pose.position.y = y_y
            pub_y_pose.position.z = y_z
            pub_y_pose.orientation.x = y_roll
            pub_y_pose.orientation.y = y_pitch
            pub_y_pose.orientation.z = y_yaw
            pub_y_pose.orientation.w = y_w

            print("y_x : ",y_x, "y_y : ", y_y,"y_z : ", y_z)
            pub_y.publish(pub_y_pose)

            # publish tf only if distance between yellow fruit is < 0.69 i.e reachable

            if y_y < 0.69:

                _status.data = 1

                pub_status.publish(_status)

                br.sendTransform((y_x, y_y, y_z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            'fruit_yellow', 'ebot_base',
                            )

                rate.sleep()             


        if len(r_center) >= 1:

            r_pose = tfBuffer.lookup_transform(
                "ebot_base", "red", rospy.Time(0))

            # print("r_pose",r_pose)

            r_x = r_pose.transform.translation.x
            r_y = r_pose.transform.translation.y
            r_z = r_pose.transform.translation.z
            r_roll = r_pose.transform.rotation.x
            r_pitch = r_pose.transform.rotation.y
            r_yaw = r_pose.transform.rotation.z
            r_w = r_pose.transform.rotation.w

            pub_r_pose.position.x = r_x
            pub_r_pose.position.y = r_y
            pub_r_pose.position.z = r_z
            pub_r_pose.orientation.x = r_roll
            pub_r_pose.orientation.y = r_pitch
            pub_r_pose.orientation.z = r_yaw
            pub_r_pose.orientation.w = r_w

            print("r_x : ", r_x ,"r_y : ", r_y ,"r_z : ", r_z )

            pub_r.publish(pub_r_pose)
            
            if r_y < 1.0  :

                _status.data = 1
                pub_status.publish(_status)  

                br.sendTransform((r_x, r_y, r_z),
                            tf.transformations.quaternion_from_euler(0, 0, 0),
                            rospy.Time.now(),
                            'fruit_red', 'ebot_base',
                            )
                rate.sleep()             
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass


def main():

    global pose_y, pose_r

    rospy.init_node("percepStack", anonymous=True)

    rgb_img = message_filters.Subscriber("/camera/color/image_raw2", Image)
    depth_img = message_filters.Subscriber("/camera/depth/image_raw2", Image)
    cam_info = message_filters.Subscriber(
        "/camera/depth/camera_info2", CameraInfo)

    clbk = message_filters.TimeSynchronizer(
        [rgb_img, depth_img, cam_info], queue_size=1)  # queue_size should be 1
    clbk.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except CvBridgeError:
        pass
    finally:
        print("Executed Perception Stack Script")
