import rospy
import tf
import numpy as numpy
import threading
import sophus as sp
from apriltag_ros.msg import AprilTagDetectionArray 
import tf.transformations as tfm
# from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, Twist
import numpy as np
from mobile_robotics.msg import WheelCmdVel

nTfRetry = 1
retryTime = 0.05


def poselist2pose(poselist):
    pose = Pose()
    pose.position.x = poselist[0]
    pose.position.y = poselist[1]
    pose.position.z = poselist[2]
    pose.orientation.x = poselist[3]
    pose.orientation.y = poselist[4]
    pose.orientation.z = poselist[5]
    pose.orientation.w = poselist[6]
    return pose

def pose2poselist(pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def xyzquat_from_matrix(matrix):
    return tfm.translation_from_matrix(matrix).tolist() + tfm.quaternion_from_matrix(matrix).tolist()


def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(tfm.compose_matrix(translate=translate) ,
                   tfm.quaternion_matrix(quaternion))

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))

def pubFrame(br, pose=[0,0,0,0,0,0,1], frame_id='obj', parent_frame_id='map', npub=1):
    if len(pose) == 7:
        ori = tuple(pose[3:7])
    elif len(pose) == 6:
        ori = tfm.quaternion_from_euler(*pose[3:6])
    else:
        print('Bad length of pose')
        return None
    
    pos = tuple(pose[0:3])
    
    for j in range(npub):
        br.sendTransform(pos, ori, rospy.Time.now(), frame_id, parent_frame_id)
        rospy.sleep(0.01)

def lookupTransform(lr, sourceFrame, targetFrame):
    for i in range(nTfRetry):
        try:
            t = rospy.Time(0)
            (trans,rot) = lr.lookupTransform(sourceFrame, targetFrame, t)
            if lr.getLatestCommonTime(sourceFrame, targetFrame) < (rospy.Time.now() - rospy.Duration(1)):
                return None
            return list(trans) + list(rot)
        except:
            print ('[lookupTransform] failed to transform targetFrame %s sourceFrame %s, retry %d' , targetFrame, sourceFrame, i)
            rospy.sleep(retryTime)
    return None

def transformPose(lr, pose, sourceFrame, targetFrame):
    _pose = PoseStamped()
    _pose.header.frame_id = sourceFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = tfm.quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()
    
    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]
    
    for i in range(nTfRetry):
        try:
            t = rospy.Time(0)
            _pose.header.stamp = t
            _pose_target = lr.transformPose(targetFrame, _pose)
            p = _pose_target.pose.position
            o = _pose_target.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except: 
            print ('[transformPose] failed to transform targetFrame %s sourceFrame %s, retry %d' % (targetFrame, sourceFrame, i))
            rospy.sleep(retryTime)
            
    return None

class ApriltagReach():
    def __init__(self, constant_vel = False):
        self.lr = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.Q = []
        

        self.apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)


        self.thread = threading.Thread(target = self.constant_vel_loop)
        self.thread.start()
        
        rospy.sleep(1)


    def apriltag_callback(self,data):
       
        for detection in data.detections:
           
            poselist_tag_cam = pose2poselist(detection.pose.pose.pose)
            poselist_tag_base = transformPose(self.lr, poselist_tag_cam, sourceFrame = '/camera_rgb_optical_frame', targetFrame = '/base_footprint')
            
            
            self.init_pos = poselist_tag_base


    def constant_vel_loop(self):

        pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rate = rospy.Rate(10)
        const_vel = Twist()
        rospy.sleep(2)
       
        U = self.Q

        const_vel.linear.x = U[0]
        const_vel.linear.y = U[1]
        const_vel.linear.z = U[2]
        const_vel.angular.x = U[3]
        const_vel.angular.y = U[4]
        const_vel.angular.z = U[5]
        rospy.sleep(1)
        
        self.velcmd_pub.publish(const_vel)
        
        print("vel cmd start")
        rospy.sleep(11)
        
        const_vel.linear.x=const_vel.linear.y=const_vel.linear.z=0.0
        const_vel.angular.x=const_vel.angular.y=const_vel.angular.z=0.0
        print("vel cmd done")
        
        self.velcmd_pub.publish(const_vel)

        # const_vel = Twist()
        

def main():
    rospy.init_node('april_reach',anonymous=True)
    april_reach = ApriltagReach()

    # get the init pose in a matrix format
    start_pos_matrix = matrix_from_xyzquat(april_reach.init_pos)

    #get a target pose in a matrix format
    des_pos = [0.0, 0.0 , 0.12, 1, 1, 1, 0 ]
    des_pos_matrix = matrix_from_xyzquat(des_pos)
    # Y = des_pos_matrix
    # X = start_pos_matrix

    # trans matrix to SE3
    X = sp.SE3(start_pos_matrix)
    print("Start pose - SE3 :")
    print(X)
    Y = sp.SE3(des_pos_matrix)
    print("Des pose in - SE3  :")
    print(Y)
    
    # Y=X*exp(Qt)
    X_1 = X.inverse()
    YX_1 = X_1*Y
    
    # log(Y*X^1) = Qt
    Log_YX_1 = YX_1.log()
    
    # t = 11s
    Q = Log_YX_1/10
    print("Dynamic Trajectory Ω:")
    print(Q)
    april_reach.Q = Q

    rospy.sleep(1)
    rospy.spin()

    
if __name__=='__main__':
    main()