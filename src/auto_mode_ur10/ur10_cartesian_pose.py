#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import geometry_msgs.msg 

def move_ur10(msg, args):
    robot = args[0]
    ur10 = args[1]
    
    end_effector = ur10.get_end_effector_link()

    ur10.allow_replanning(True)

    ur10.set_goal_position_tolerence(0.01)
    ur10.set_goal_orientation_tolerence(0.1)

    desired_pose = msg.position
    print(desired_pose)
    print('========================================')

    waypoint = []
    start_point = ur10.get_current_pose(end_effector).pose
    poseTarget = copy.deepcopy(start_point)

    poseTarget.position.x = 0
    poseTarget.position.y = 0
    poseTarget.position.z = 0

    waypoint.append(copy.deepcopy(posetarget))

    ur10.set_start_state_to current_state()

    if np.sqrt((poseTarget.position.x - start_point.position.x)**2 +
               (poseTarget.position.y - start_point.position.y)**2 +
               (poseTarget.position.z - start_point.position.z)**2) < 0.1:
        rospy.loginfo("Warnig: target position overlaps with the initial position!")


    (plan, fracion) = ur10.compute_cartesian_path(
        waypoint,
        0.01,
        0.0
        True
    )

    if 1- fraction <0.2:
        rospy.loginfo("path computed successfull.Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        rospy.loginfo("\n# intermediate waypoints = " + str(num_pts))

        print('===== MOVING =====')
        ur10.execute(plan, wait=True)
        rospy.loginfo("Path execution complete.")
        rospy.loginfo('followed ' + str(fraction*100) + '% of requested trajectory')  #fraction: rate of successful requested

    else:
        rospy.logininfo("path planning failed")

    print('==== ROBOT STATE AFTER MOVING ====')
    print(robot.get_current_state().joint_state.position)
    print('=============')

    print('==== EE POSE ====')
    print(ur5.get_current_pose().pose)
    print('=============')

    def subscriber():
        print()
        print('=============================================================')

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur10_cartesian_pose', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.planningSceneInterface()

        ur10 = moveit_commander.MovegroupCommander('manipulator')

        print('==================Refrence frame:', ur5.get_planning_frame())
        print('======= End Effector:', ur5.get_end_effector_link())
        # Set the reference frame for pose targets
        reference_frame = "/base_link"
        ur5.set_pose_reference_frame(reference_frame)

        print('======= ROBOT STATE =======')
        print(robot.get_current_state().joint_state.position)
        print('================')

        print('==== EE POSE ====')
        print(ur5.get_current_pose().pose)
        print('=============')

        rospy.Subscriber('hololens', geometry_msgs.msg.Pose, move_ur10, (robot, ur10), queue_size = 1)
        rospy.spin()

    if__name__ == '__main__':
        try:
            subscriber()
        
        except rospy.ROSInterruptException:
        # When finished shut down moveit_commander.
        moveit_commander.roscpp_shutdown()
        print('==== STOPPING ====')
        pass


