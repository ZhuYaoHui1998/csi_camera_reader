#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def gstreamer_pipeline(sensor_id=0, sensor_mode=0, capture_width=1920, capture_height=1080, framerate=20):
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! "
        f"video/x-raw(memory:NVMM),width={capture_width},height={capture_height},framerate={framerate}/1,format=NV12 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink"
    )
    rospy.loginfo("GStreamer pipeline: %s", pipeline)
    return pipeline

def publish_camera_stream(sensor_id):
    topic_name = f'/csi_camera_{sensor_id}/image_raw'
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    bridge = CvBridge()

    pipeline = gstreamer_pipeline(sensor_id=sensor_id, sensor_mode=0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        rospy.logerr("Error: Unable to open camera with sensor ID %d", sensor_id)
        return

    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Error: Unable to read frame from camera with sensor ID %d", sensor_id)
            break

        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        image_msg.header.stamp = rospy.Time.now()
        image_msg.header.frame_id = f"camera_{sensor_id}_frame"
        image_pub.publish(image_msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    rospy.init_node('csi_camera_node', anonymous=True)
    sensor_id = rospy.get_param('~sensor_id', 0)
    rospy.loginfo("Starting camera with sensor ID %d", sensor_id)
    try:
        publish_camera_stream(sensor_id)
    except rospy.ROSInterruptException:
        pass
