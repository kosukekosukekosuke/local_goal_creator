#!/usr/bin/python3

import rospy
import math
import tf_conversions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# from ros_gym_sfm.msg import two_dimensional_position

class LocalGoalCreator:
    def __init__(self):
        # parameter
        self.HZ = rospy.get_param("/HZ")
        self.PROCESS_1_FLAG = rospy.get_param("/PROCESS_1_FLAG")
        self.PROCESS_2_FLAG = rospy.get_param("/PROCESS_2_FLAG")
        self.GLOBAL_PATH_INDEX = rospy.get_param("/GLOBAL_PATH_INDEX")
        self.GOAL_INDEX = rospy.get_param("/GOAL_INDEX")
        self.LOCAL_GOAL_DIST = rospy.get_param("/LOCAL_GOAL_DIST")
        self.LOCAL_GOAL_UPDATE_STEP = rospy.get_param("/LOCAL_GOAL_UPDATE_STEP")
        self.GLOBAL_PATH_SPLIT = rospy.get_param("/GLOBAL_PATH_SPLIT")
        self.GOAL_INDEX_2 = rospy.get_param("/GOAL_INDEX_2")

        # create instance
        self.agent_pose = PoseStamped()
        self.agent_goal = PoseStamped()
        self.global_path = Path()
        self.local_goal = PoseStamped()

        # frame_id
        self.local_goal.header.frame_id = "map"

        # subscriber
        self.agent_pose_sub = rospy.Subscriber("ros_gym_sfm/agent_pose", PoseStamped, self.agent_pose_callback)
        self.agent_goal_sub = rospy.Subscriber("ros_gym_sfm/agent_goal", PoseStamped, self.agent_goal_callback)    

        # publisher
        self.global_path_pub = rospy.Publisher("ros_gym_sfm/global_path", Path, queue_size=1)  #for rviz debug
        self.local_goal_pub = rospy.Publisher("ros_gym_sfm/local_goal", PoseStamped, queue_size=1)  

    # callback
    def agent_pose_callback(self, agent_pose):
        self.agent_pose.pose.position.x = agent_pose.pose.position.x
        self.agent_pose.pose.position.y = agent_pose.pose.position.y
        self.pose_moving_flag = True

    def agent_goal_callback(self, agent_goal):
        self.agent_goal.pose.position.x = agent_goal.pose.position.x
        self.agent_goal.pose.position.y = agent_goal.pose.position.y
        self.goal_moving_flag = True

    # make_global_path
    def make_global_path_1(self, current_pose: PoseStamped, goal_pose: PoseStamped) -> Path:
        global_path = Path()
        global_path.header.frame_id = "map"
        global_path.header.stamp = rospy.Time.now()

        dx = (goal_pose.pose.position.x - current_pose.pose.position.x) / (self.GLOBAL_PATH_INDEX - 1)
        dy = (goal_pose.pose.position.y - current_pose.pose.position.y) / (self.GLOBAL_PATH_INDEX - 1)

        for i in range(self.GLOBAL_PATH_INDEX):
            pose = PoseStamped()

            pose.pose.position.x = current_pose.pose.position.x + dx * i
            pose.pose.position.y = current_pose.pose.position.y + dy * i

            global_path.poses.append(pose)

        return global_path
    
    def make_global_path_2(self, current_pose: PoseStamped, goal_pose: PoseStamped) -> Path:
        global_path = Path()
        global_path.header.frame_id = "map"
        global_path.header.stamp = rospy.Time.now()

        dx = goal_pose.pose.position.x - current_pose.pose.position.x
        dy = goal_pose.pose.position.y - current_pose.pose.position.y
        self.global_path_index = int(math.hypot(dx, dy) // self.GLOBAL_PATH_SPLIT)
        
        # If it is less than or equal to 2, the current position and goal are used as global path
        if self.global_path_index > 2 :
            small_dx = dx / (self.global_path_index - 1)
            small_dy = dy / (self.global_path_index - 1)        
            for i in range(self.global_path_index):
                pose = PoseStamped()
                pose.pose.position.x = current_pose.pose.position.x + small_dx * i
                pose.pose.position.y = current_pose.pose.position.y + small_dy * i
                global_path.poses.append(pose)
        else:
            global_path.poses.append(current_pose)
            global_path.poses.append(goal_pose)
           
        return global_path   
    
    # initialize_local_goal
    def initialize_local_goal_1(self, local_goal: PoseStamped) -> PoseStamped:
        local_goal.pose.position.x = self.global_path.poses[self.GOAL_INDEX].pose.position.x
        local_goal.pose.position.y = self.global_path.poses[self.GOAL_INDEX].pose.position.y
        self.initialize_local_goal_flag = False

        return local_goal
    
    def initialize_local_goal_2(self, local_goal: PoseStamped) -> PoseStamped:
        local_goal.pose.position.x = self.global_path.poses[self.GOAL_INDEX_2].pose.position.x
        local_goal.pose.position.y = self.global_path.poses[self.GOAL_INDEX_2].pose.position.y
        self.initialize_local_goal_flag = False

        return local_goal

    # make_local_goal
    def make_local_goal_1(self, current_pose: PoseStamped, local_goal: PoseStamped) -> PoseStamped:
        dx = current_pose.pose.position.x - self.global_path.poses[self.GOAL_INDEX].pose.position.x
        dy = current_pose.pose.position.y - self.global_path.poses[self.GOAL_INDEX].pose.position.y
        dist = math.hypot(dx, dy)

        if dist < self.LOCAL_GOAL_DIST:
            self.GOAL_INDEX += self.LOCAL_GOAL_UPDATE_STEP

            if self.GOAL_INDEX < len(self.global_path.poses):
                local_goal.pose.position.x = self.global_path.poses[self.GOAL_INDEX].pose.position.x
                local_goal.pose.position.y = self.global_path.poses[self.GOAL_INDEX].pose.position.y
            else:
                self.GOAL_INDEX = len(self.global_path.poses) - 1
                local_goal.pose.position.x = self.global_path.poses[self.GOAL_INDEX].pose.position.x
                local_goal.pose.position.y = self.global_path.poses[self.GOAL_INDEX].pose.position.y

        return local_goal 

    def make_local_goal_2(self, local_goal: PoseStamped) -> PoseStamped:
        if self.GOAL_INDEX_2 < self.global_path_index : 
            local_goal.pose.position.x = self.global_path.poses[self.GOAL_INDEX_2].pose.position.x
            local_goal.pose.position.y = self.global_path.poses[self.GOAL_INDEX_2].pose.position.y
        else:
            self.GOAL_INDEX_2 = self.global_path_index - 1
            local_goal.pose.position.x = self.global_path.poses[self.global_path_index - 1].pose.position.x
            local_goal.pose.position.y = self.global_path.poses[self.global_path_index - 1].pose.position.y
            self.make_local_goal_flag = False

        return local_goal 

    # Process 1
    # global path : Generate only the first time
    # local goal  : Updated when the distance between the current position and local goal is close
    def process_1(self):
        if self.making_path_flag == False:
            self.global_path = self.make_global_path_1(self.agent_pose, self.agent_goal)
            self.making_path_flag = True

        self.global_path_pub.publish(self.global_path)

        if self.initialize_local_goal_flag == True:
            self.local_goal = self.initialize_local_goal_1(self.local_goal)

        self.local_goal = self.make_local_goal_1(self.agent_pose, self.local_goal)
        self.local_goal.pose.orientation.w = 1.0
        local_goal_q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
        self.local_goal.pose.orientation.x = local_goal_q[0]
        self.local_goal.pose.orientation.y = local_goal_q[1]
        self.local_goal.pose.orientation.z = local_goal_q[2]
        self.local_goal.pose.orientation.w = local_goal_q[3]

        self.local_goal.header.stamp = rospy.Time.now()
        self.local_goal_pub.publish(self.local_goal)

    # Process 2
    # global path : Always update
    # local goal  : Keep placing local goal on the global path
    def process_2(self):
        self.global_path = self.make_global_path_2(self.agent_pose, self.agent_goal)

        self.global_path_pub.publish(self.global_path)

        if self.initialize_local_goal_flag == True:
            self.local_goal = self.initialize_local_goal_2(self.local_goal)
        
        if self.make_local_goal_flag == True:
            self.local_goal = self.make_local_goal_2(self.local_goal)
            self.local_goal.pose.orientation.w = 1.0
            local_goal_q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
            self.local_goal.pose.orientation.x = local_goal_q[0]
            self.local_goal.pose.orientation.y = local_goal_q[1]
            self.local_goal.pose.orientation.z = local_goal_q[2]
            self.local_goal.pose.orientation.w = local_goal_q[3]

        self.local_goal.header.stamp = rospy.Time.now()
        self.local_goal_pub.publish(self.local_goal)

    def process(self):
        rate = rospy.Rate(self.HZ)
        self.pose_moving_flag = False
        self.goal_moving_flag = False
        self.making_path_flag = False
        self.initialize_local_goal_flag = True
        self.make_local_goal_flag = True
        self.global_path_index = 0

        while not rospy.is_shutdown():
            if self.pose_moving_flag == True and self.goal_moving_flag == True:
                if self.PROCESS_1_FLAG == True:
                    self.process_1()

                if self.PROCESS_2_FLAG == True:
                    self.process_2()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("local_goal_creator", anonymous=True)
    # rospy.loginfo("hello ros")   # check to see if ros is working

    local_goal_creator = LocalGoalCreator()

    try:
        local_goal_creator.process()

    except rospy.ROSInterruptException:
        pass