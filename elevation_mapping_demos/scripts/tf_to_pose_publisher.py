#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
import tf

def callback(newPose):
    """Listens to a transform between from_frame and to_frame and publishes it
       as a pose with a zero covariance."""
    global publisher, tf_listener, from_frame, to_frame

    # Listen to transform and throw exception if the transform is not
    # available.
    try:
        (trans, rot) = tf_listener.lookupTransform(
            from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return

    # Create and fill pose message for publishing
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
    global publisher, tf_listener, from_frame, to_frame
    rospy.init_node('tf_to_pose_publisher')
    # Read frame id's for tf listener
    from_frame = rospy.get_param("~from_frame")
    to_frame = rospy.get_param("~to_frame")
    pose_name = str(to_frame) + "_pose"

    tf_listener = tf.TransformListener()
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
