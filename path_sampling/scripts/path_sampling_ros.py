import rospy
from nav_msgs.msg import Path, Odometry
from navfn.srv import MakeNavPlan
from std_msgs.msg import Header
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose2D, PoseStamped, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from path_sampling import PathSampler

import numpy as np


class PathSamplerROS():
    def __init__(self, name="PathSampler"):
        self.name = name

        print(f"[{self.name}] Iniciating node")

        self._waypoint_dst = rospy.get_param("global_plan_waypoint_dst", 2)
        self._visualize_wp = rospy.get_param("visualize_waypoints", True)
        self._base_frame   = rospy.get_param("base_frame", "base_link")
        self._map_frame    = rospy.get_param("map_frame", "map")

        self._mka_pub  = rospy.Publisher("/path_sampler/waypoints_vis", MarkerArray, queue_size=2)
        self._wp_pub   = rospy.Publisher("/path_sampler/waypoints", Path, queue_size=10)

        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb)

        self._listener = tf.TransformListener()

        self.setup()

        self._sampler = PathSampler(waypoint_dst=self._waypoint_dst)

    def setup(self):

        self._plan_srv = rospy.ServiceProxy("/navfn/make_plan", MakeNavPlan)
        self._plan_srv.wait_for_service()
        print(f"[{self.name}] Global planner server is ready!")

    def cb(self, target):
        self.get_waypoints(target, method='cv')

    def get_waypoints(self, target, method='u'):
        start = self.get_robot_pose()
        # try:
        plan = self._plan_srv(start, target)

        if method == 'u':
            wps = self._sampler.sample_plan_uniform(self.path_to_numpy(plan.path))

        elif method == 'cv':
            wps = self._sampler.sample_curve_from_gradient(self.path_to_numpy(plan.path))
            

        if self._visualize_wp:
            self.visualize_waypoints(wps)
        
        return wps
        
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        
        # except:
        #     print(f"{[self.name]} No path found")

        return None
    
    def get_robot_pose(self):
        try:
            (pos, rot) = self._listener.lookupTransform(self._map_frame, self._base_frame, rospy.Time(0))
            pose = PoseStamped()
            pose.header.stamp = rospy.Time().now()
            pose.header.frame_id = self._map_frame
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            return pose
         
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get robot pose")
            return None
    
    @staticmethod
    def build_pose_stamped(pose_2d):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time().now()
        pose.header.frame_id = 'map'

        pose.pose.position.x = pose_2d.x
        pose.pose.position.y = pose_2d.y

        quaternion = quaternion_from_euler(0, 0, pose_2d.theta)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose
    
    
    @staticmethod
    def get_yaw(q: Quaternion):
        q_l = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(q_l)

        return yaw

    def path_to_numpy(self, path: Path):
        pt_ = []
        for i in range(len(path)):
            pt_.append([path[i].pose.position.x, 
                       path[i].pose.position.y, 
                       self.get_yaw(path[i].pose.orientation)])            

        return np.asarray(pt_)

    def visualize_waypoints(self, points, color=[1, 1, 0, 0]):
        marker_array = MarkerArray()

        index = 0
        for wp in points:
            header = Header()
            header.frame_id = self._map_frame
            header.stamp = rospy.Time.now()
            marker = Marker(header = header,
                            ns = "waypoints",
                            id = index,
                            type = Marker.SPHERE,
                            action = Marker.ADD,
                            scale = Vector3(x=.1, y=.1, z=0.01),
                            lifetime = rospy.Duration(5))
            
            marker.color.a = color[0]
            marker.color.r = color[1]
            marker.color.g = color[2]
            marker.color.b = color[3]

            index += 1
            marker.pose = self.build_pose_stamped(Pose2D(*wp)).pose
            marker_array.markers.append(marker)

        self._mka_pub.publish(marker_array)