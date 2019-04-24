#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs
import tf

def callback(newPose):
    global publisher, tfListener, from_frame, to_frame

    try:
        (trans, rot) = tfListener.lookupTransform(from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = from_frame
    pose.pose.pose.position.x = trans[0]
    pose.pose.pose.position.y = trans[1]
    pose.pose.pose.position.z = trans[2]
    pose.pose.pose.orientation.x = rot[0]
    pose.pose.pose.orientation.y = rot[1]
    pose.pose.pose.orientation.z = rot[2]
    pose.pose.pose.orientation.w = rot[3]

    pose.pose.covariance = [0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0, 
                  0, 0, 0, 0, 0, 0]
    
    publisher.publish(pose)

    
#Main function initializes node and subscribers and starts the ROS loop
def main_program():
    global publisher, tfListener, from_frame, to_frame
    rospy.init_node('tf_to_pose_publisher')

    from_frame = rospy.get_param("waffle_pose_publisher/from_frame")
    to_frame = rospy.get_param("waffle_pose_publisher/to_frame")
    pose_name = str(to_frame) + "_pose"

    tfListener = tf.TransformListener()
    publisher = rospy.Publisher(pose_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException: pass
