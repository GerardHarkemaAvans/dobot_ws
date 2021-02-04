#!/usr/bin/env python


from binpicking_vision.srv import CapturePointcloud, CapturePointcloudRequest, CapturePointcloudResponse
import rospy
import copy
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf_conversions

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

pcl_publisher = 0
capture = 0
tfBuffer = 0
listener = 0
pcl_publisher = 0
broadcaster = 0
transform_capture = 0

def capture_cb(request):
    rospy.loginfo('capture_pointcloud_server: data found copying....')
    global capture
    capture = copy.deepcopy(request)
    rospy.loginfo('capture_pointcloud_server: data found copying....')



#    pcl_publisher.publish(capture)

# Service callback function.
def process_service_request(req):

    # Instantiate the response message object.
    res = CapturePointcloudResponse()
    res.success = False

    rospy.loginfo('capture_pointcloud_server: Started Capturing')
    sb = rospy.Subscriber('/camera' + str(req.camera) +'/depth/color/points', PointCloud2, capture_cb)
    rospy.wait_for_message('/camera'+ str(req.camera) +'/depth/color/points', PointCloud2)
    sb.unregister()

    #Return the response message.
#    global transform_capture
    global capture
    res.pointcloud = capture
#    res.camera_pose.position = transform_capture.transform.translation
#    res.camera_pose.orientation = transform_capture.transform.rotation
    res.success = True
    return res

def capture_pointcloud_server():
    # ROS node for the service server.
    rospy.init_node('capture_pointcloud_server', anonymous = False,log_level=rospy.DEBUG)
    rospy.loginfo('capture_pointcloud_server: Starting')

    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    global listener
    listener = tf2_ros.TransformListener(tfBuffer)

    #If you want to see the captured vision uncommend this code
    #global pcl_publisher
    #pcl_publisher = rospy.Publisher('/camera2/depth/color/capture', PointCloud2, queue_size = 10)

    global broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a ROS service type.
    service = rospy.Service('capture_pointcloud', CapturePointcloud, process_service_request)

    # Log message about service availability.
    rospy.loginfo('capture_pointcloud_server: Capture pointcluod service is now available.')
    rospy.spin()

if __name__ == "__main__":
    capture_pointcloud_server()
