#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf2_ros

def callback(newPose):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_buffer, tf_listener, from_frame, to_frame

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        trans = tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return

    # Create and fill pose message for publishing
    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header = trans.header
    pose.pose.pose.position = trans.transform.translation
    pose.pose.pose.orientation = trans.transform.rotation

    # Since tf transforms do not have a covariance, pose is filled with
    # a zero covariance.
    pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0]

    publisher.publish(pose)


def main_program():
    """ Main function initializes node and subscribers and starts
        the ROS loop. """
    global publisher, tf_buffer, tf_listener, from_frame, to_frame
    rospy.init_node('tf_to_pose_publisher')
    # Read frame id's for tf listener
    from_frame = rospy.get_param("~from_frame")
    to_frame = rospy.get_param("~to_frame")
    pose_name = str(to_frame) + "_pose"

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    publisher = rospy.Publisher(
        pose_name, geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

    # Set callback and start spinning
    rospy.Timer(rospy.Duration(0.05), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main_program()
    except rospy.ROSInterruptException:
        pass
