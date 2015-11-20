#!/usr/bin/python
# Script to merge poses for single AR board
#
# Run as:
#   $ rosrun ar_sys merge_poses_for_single_board.py _poses:="pose_topic_1, pose_topic_2, ..." _output_pose:=<output topic>
#
# Author: Marynel Vazquez
# Creation Date: 11/19/15

import threading
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
from tf import TransformListener
from tf.transformations import *
from math import *
import numpy

def avgAngles(a,b,w1=0.5,w2=0.5):
    C = w1*cos(a) + w2*cos(b)
    S = w1*sin(a) + w2*sin(b)
    return atan2(S,C)

class MergePoses:

    def __init__(self):

        self.avg_pose = None
        self.tl = TransformListener()

        self.topics = rospy.get_param('~poses',[])
        print self.topics
        if len(self.topics) == 0:
            rospy.logerr('Please provide a poses list as input parameter')
            return
        self.output_pose = rospy.get_param('~output_pose','ar_avg_pose')
        self.output_frame = rospy.get_param('~output_frame', 'rf_map')
        
        self.subscribers = []
        for i in self.topics:
            subi = rospy.Subscriber(i, PoseStamped, self.callback, queue_size=1)
            self.subscribers.append(subi)

        self.pub = rospy.Publisher(self.output_pose, PoseStamped, queue_size=1)
            
        self.mutex_avg = threading.Lock()
        self.mutex_t = threading.Lock()
        
        self.transformations = {}
        
    def callback(self, pose_msg):

        # get transformation to common frame
        position = None
        quaternion = None
        if self.transformations.has_key(pose_msg.header.frame_id):
            position, quaternion = self.transformations[pose_msg.header.frame_id]
        else:
            if self.tl.frameExists(pose_msg.header.frame_id) and \
            self.tl.frameExists(self.output_frame):
                t = self.tl.getLatestCommonTime(self.output_frame, pose_msg.header.frame_id)
                position, quaternion = self.tl.lookupTransform(self.output_frame,
                                                               pose_msg.header.frame_id, t)
                self.mutex_t.acquire()
                self.transformations[pose_msg.header.frame_id] = (position, quaternion)
                self.mutex_t.release()
                rospy.loginfo("Got static transform for %s" % pose_msg.header.frame_id)

        # transform pose

        framemat = self.tl.fromTranslationRotation(position, quaternion)

        posemat = self.tl.fromTranslationRotation([pose_msg.pose.position.x,
                                                   pose_msg.pose.position.y,
                                                   pose_msg.pose.position.z],
                                                  [pose_msg.pose.orientation.x,
                                                   pose_msg.pose.orientation.y,
                                                   pose_msg.pose.orientation.z,
                                                   pose_msg.pose.orientation.w])

        newmat = numpy.dot(framemat,posemat)
        
        xyz = tuple(translation_from_matrix(newmat))[:3]
        quat = tuple(quaternion_from_matrix(newmat))

        tmp_pose = PoseStamped()
        tmp_pose.header.stamp = pose_msg.header.stamp
        tmp_pose.header.frame_id = self.output_frame
        tmp_pose.pose = Pose(Point(*xyz),Quaternion(*quat))
        
        # self.tl.waitForTransform(self.output_frame, pose_msg.header.frame_id, rospy.Time(), rospy.Duration(1.0))
        # tmp_pose = self.tl.transformPose(self.output_frame,pose_msg)

        tmp_angles = euler_from_quaternion([tmp_pose.pose.orientation.x,
                                            tmp_pose.pose.orientation.y,
                                            tmp_pose.pose.orientation.z,
                                            tmp_pose.pose.orientation.w])
        
        # compute average
        self.mutex_avg.acquire()
        
        if self.avg_pose == None or (pose_msg.header.stamp - self.avg_pose.header.stamp).to_sec() > 0.5:
            self.avg_pose = tmp_pose
        else:

            print (pose_msg.header.stamp - self.avg_pose.header.stamp).to_sec()
            
            self.avg_pose.header.stamp = pose_msg.header.stamp
            a = 0.7
            b = 0.3
            self.avg_pose.pose.position.x = a*self.avg_pose.pose.position.x + b*tmp_pose.pose.position.x
            self.avg_pose.pose.position.y = a*self.avg_pose.pose.position.y + b*tmp_pose.pose.position.y
            self.avg_pose.pose.position.z = a*self.avg_pose.pose.position.z + b*tmp_pose.pose.position.z

            angles = euler_from_quaternion([self.avg_pose.pose.orientation.x,
                                            self.avg_pose.pose.orientation.y,
                                            self.avg_pose.pose.orientation.z,
                                            self.avg_pose.pose.orientation.w])
            angles = list(angles)
            angles[0] = avgAngles(angles[0], tmp_angles[0], 0.7, 0.3)
            angles[1] = avgAngles(angles[1], tmp_angles[1], 0.7, 0.3)
            angles[2] = avgAngles(angles[2], tmp_angles[2], 0.7, 0.3)

            q = quaternion_from_euler(angles[0], angles[1], angles[2])
            
            self.avg_pose.pose.orientation.x = q[0]
            self.avg_pose.pose.orientation.y = q[1]
            self.avg_pose.pose.orientation.z = q[2]
            self.avg_pose.pose.orientation.w = q[3]

        self.pub.publish(self.avg_pose)
            
        self.mutex_avg.release()

if __name__ == '__main__':
    rospy.init_node('merge_poses_for_single_board', anonymous=True)
    merger = MergePoses()
    rospy.spin()
