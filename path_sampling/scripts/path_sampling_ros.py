import rospy
from nav_msgs.msg import Path, Odometry
from navfn.srv import MakeNavPlan
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose2D, PoseStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np



class PathSamplerROS():
    def __init__(self, name="ExecuteGlobalPlan"):

        print(f"[{self.name}] Iniciating node")

        self._waypoint_dst = rospy.get_param("global_plan_waypoint_dst", 2)
        self._visualize_wp = rospy.get_param("visualize_waypoints", True)
        self._base_frame   = rospy.get_param("base_frame", "base_link")
        self._map_frame    = rospy.get_param("map_frame", "map")

        self._mka_pub  = rospy.Publisher("/path_sampler/waypoints_vis", MarkerArray, queue_size=2)
        self._wp_pub   = rospy.Publisher("/path_sampler/waypoints", Path, queue_size=10)

        self._listener = tf.TransformListener()


    def setup(self):

        self._plan_srv = rospy.ServiceProxy("/navfn/make_plan", MakeNavPlan)
        self._plan_srv.wait_for_service()
        print(f"[{self.name}] Global planner server is ready!")
        
    def initialise(self):

        self.goal_pub = rospy.Publisher("coverage_navigation/current_goal", PoseStamped, queue_size=10)
        self.setup()
        
    # def execute_plan(self, start_pose):
        
    #     start_pose_stamped = self.build_pose_stamped(start_pose)
    #     plan = self.get_waypoints(start_pose_stamped)

    #     if plan is not None:
    #         print(f"[{self.name}] Got plan with {len(plan)} points")
    #         for i in range(len(plan)):

    #             goal_0 = plan[i]
    #             self.goal_pub.publish(goal_0)

    #             goal = RLControllerGoal()

    #             goal.navigation_target.pose.position.x = goal_0.pose.position.x
    #             goal.navigation_target.pose.position.y = goal_0.pose.position.y
                
    #             self.rl_client.send_goal(goal)
    #             print(f"sending goal {i} to planner server")

    #             self.rl_client.wait_for_result()
    #             print("got result!")
    #             i += 1

    #         print(f"[{self.name}] Path had been executed succesfully!")
    #     else:
    #         print(f"[{self.name}] Could not find global path!!")

    def get_waypoints(self, target):
        start = self.get_robot_pose()
        try:
            plan = self._plan_srv(start, target)
            wps = self.sample_plan_uniform(plan.path)

            if self._visualize_wp:
                self.visualize_waypoints(wps)
            
            return wps
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        except:
            print(f"[self.name] No path found")

        return None
    
    def sample_plan_uniform(self, path):
        L = 0
        waypoints = []
        for i in range(1, len(path)):
            segment_l = self.calculate_segment_lenght(
                [path[i-1].pose.position.x, path[i-1].pose.position.y],
                [path[i].pose.position.x, path[i].pose.position.y]
            ) 
            L += segment_l
            
            if L >= self._waypoint_dst:
                waypoints.append(path[i])
                L -= self._waypoint_dst

        if L < self._waypoint_dst and len(waypoints)>0:
            waypoints.pop()
        waypoints.append(path[-1])

        return waypoints

    def sample_plan_curvature(self, path):
        pass


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
    def calculate_segment_lenght(s1, s2):
        x1, y1 = s1
        x2, y2 = s2
        return np.sqrt( (x1 - x2)**2 + (y1 - y2)**2 )

    def visualize_waypoints(self, points, color=[1, 1, 0, 0]):
        marker_array = MarkerArray()

        index = 0
        for wp in points:
            marker = Marker(header = wp.header,
                            ns = "waypoints_osm",
                            id = index,
                            type = Marker.SPHERE,
                            action = Marker.ADD,
                            scale = Vector3(x=.1, y=.1, z=0.01),
                            lifetime = rospy.Duration(30))
            
            marker.color.a = color[0]
            marker.color.r = color[1]
            marker.color.g = color[2]
            marker.color.b = color[3]

            index += 1
            marker.pose = wp.pose
            marker_array.markers.append(marker)

        self._mka_pub.publish(marker_array)