import rospy
import sys
import moveit_commander
import math
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True


def traj_parser(robot_traj):
    """
    """
    """
    - RobotTrajectory
        - trajectory_msgs/JointTrajectory joint_trajectory
            - Header header
                - uint32 seq
                - time stamp
                    - time data
                - string frame_id
            - string[] joint_names
            - JointTrajectoryPoint[] points
                - float64[] positions
                - float64[] velocities
                - float64[] accelerations
                - float64[] effort
                - duration time_from_start
                    - secs
                    - nsecs //nanoseconds
        - trajectory_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory
            - geometry_msgs/Transform[] transforms
                - Vector3 translation
                - Quaternion rotation
            - geometry_msgs/Twist[] velocities
                - Vector3  linear
                - Vector3  angular
            - geometry_msgs/Twist[] accelerations
    """
    # joint_trajectory parser
    """
    e.g, joint_trajectory.points[0]:
    positions: [-9.985654605552555e-05, 6.981462850235404e-05, 1.9801191147416833e-05, -2.3160347668454046e-05, 1.3062500068917866e-05, -9.652617783285677e-05]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: [0.9784111031387357, 0.0, 0.0, 0.0, 0.0, 0.0]
    effort: []
    time_from_start:
      secs: 0
      nsecs:         0
    """
    joints_traj = robot_traj.joint_trajectory
    joints_name = joints_traj.joint_names
    joints_traj_points = joints_traj.points
    traj_points_length = len(joints_traj_points)
    joint_traj_time_from_starts = list()
    joint_traj_joints_positions = list()
    joint_traj_joints_velocities = list()
    joint_traj_joints_accelerations = list()
    joint_traj_joints_efforts = list()
    for joints_traj_point in joints_traj_points:
        joint_traj_time_from_starts.append(
            joints_traj_point.time_from_start.secs + joints_traj_point.time_from_start.nsecs / 1e9)
        joint_traj_joints_positions.append(joints_traj_point.positions)
        joint_traj_joints_velocities.append(joints_traj_point.velocities)
        joint_traj_joints_accelerations.append(joints_traj_point.accelerations)
        joint_traj_joints_efforts.append(joints_traj_point.effort)

    # multi_dof_joint_trajectory
    with open("traj_joint_position.csv", 'w+') as traj_position_file:
        traj_position_file.write("time,")
        for i in range(len(joints_name)):
            traj_position_file.write(joints_name[i])
            traj_position_file.write(',')
        traj_position_file.write("\n")
        for i in range(traj_points_length):
            traj_position_file.write(str(joint_traj_time_from_starts[i])+",")  # time
            for j in range(len(joints_name)):
                traj_position_file.write(str(joint_traj_joints_positions[i][j]))
                traj_position_file.write(',')
            traj_position_file.write("\n")

    with open("traj_joint_velocity.csv", 'w+') as traj_velocity_file:
        traj_velocity_file.write("time,")
        for i in range(len(joints_name)):
            traj_velocity_file.write(joints_name[i])
            traj_velocity_file.write(',')
        traj_velocity_file.write("\n")
        for i in range(traj_points_length):
            traj_velocity_file.write(str(joint_traj_time_from_starts[i])+",")  # time
            for j in range(len(joints_name)):
                traj_velocity_file.write(str(joint_traj_joints_velocities[i][j]))
                traj_velocity_file.write(',')
            traj_velocity_file.write("\n")

    with open("traj_joint_acceleration.csv", 'w+') as traj_acceleration_file:
        traj_acceleration_file.write("time,")
        for i in range(len(joints_name)):
            traj_acceleration_file.write(joints_name[i])
            traj_acceleration_file.write(',')
        traj_acceleration_file.write("\n")
        for i in range(traj_points_length):
            traj_acceleration_file.write(str(joint_traj_time_from_starts[i])+",")  # time
            for j in range(len(joints_name)):
                traj_acceleration_file.write(str(joint_traj_joints_velocities[i][j]))
                traj_acceleration_file.write(',')
            traj_acceleration_file.write("\n")


class Moveit_Arm_Planner(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.joint_plan_valid_limit = 0.1/180*math.pi
        rospy.init_node('move_group_plan', anonymous=True)
        self.arm_group_name = "arm"  # name of arm group
        self.tool_group_name = "tool"  # name of tool group
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.tool_group = moveit_commander.MoveGroupCommander(self.tool_group_name)
        # self.arm_group.set_named_target("home")
        # self.arm_group.go()
        self.planned_traj = None

    @staticmethod
    def sleep(self, duration_time):
        """
        :param duration_time: duration: seconds (or rospy.Duration) to sleep
        :return:
        """
        rospy.sleep(duration_time)

    def jointspace_plan(self, joint_target):
        """
        :param joint_target: joint position list, e.g [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        :return:
        """
        self.planned_traj = self.arm_group.plan(joint_target)  # planned_traj:RobotTrajectory
        # print(self.planned_traj.multi_dof_joint_trajectory)
        planned_joints_position = self.planned_traj.joint_trajectory.points[-1].positions
        print(planned_joints_position[0]-math.pi/2)
        is_plan_valid = all_close(joint_target, planned_joints_position, self.joint_plan_valid_limit)
        if not is_plan_valid:
            return None
        return self.planned_traj

    def shutdown(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    arm_planner = Moveit_Arm_Planner()
    traj = arm_planner.jointspace_plan([math.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0])
    # arm_planner.jointspace_plan([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    traj_parser(traj)
    arm_planner.shutdown()
