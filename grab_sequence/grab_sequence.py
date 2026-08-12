import rclpy
import time
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

def main():

    rclpy.init() # starts up ROS2 - has to run before anything else ROS-related works

    moveit = MoveItPy(node_name= "grab_sequence_node") # creates a MoveIt2 interface object and names it "grab_sequence_node" (this will show up in ros2 node list) 
    time.sleep(10.0)
    arm = moveit.get_planning_component("manipulator") # grabs reference to the "manipulator" planning group
    gripper = moveit.get_planning_component("gripper") # grabs reference to the "gripper" planning group

    def move_arm_to_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0): # takes a target position (x, y, z) and target orientation in quaternion (qx, qy, qz, qw)
        pose = PoseStamped() # creates an empty pose message
        pose.header.frame_id = "base_link" # sets the frame of reference of the input coordinates to be relative to the "base link"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        # passes in the function parameters into the "poseStamped" message fields

        arm.set_start_state_to_current_state() # sets the starting position to be the current position
        arm.set_goal_state(pose_stamped_msg=pose, pose_link="end_effector_link") # sets the goal position as the input coordinates
        plan_result = arm.plan() # tells MoveIt to run its planning pipeline (IK + collision-checking + trajectory generation) and returns a result (basically boolean) object

        if plan_result:
            success = moveit.execute(plan_result.trajectory, controllers=[])    # if planning succeeded, send the computed trajectory to the controllers to actually execute it
                                                                                # "controllers=[]" means to let MoveIt figure out which controllers to use to execute the task
            if not success:
                print("Execution failed",flush = True)
            time.sleep(0.5)
        else:
            print("Planning failed", flush=True) # if no valid plan (unreachable, collision, etc) print a message

    def set_gripper(named_state): # takes a predefined named joint configuration from SRDF (either "open" or "closed") 
        gripper.set_start_state_to_current_state() # sets the starting state to be the current state
        gripper.set_goal_state(configuration_name = named_state) # perform the gripper action
        plan_result = gripper.plan() # tells MoveIt to run its planning pipeline
        if plan_result:
            moveit.execute(plan_result.trajectory, controllers=[]) # if planning succeeded, sent the trajectory to the controllers to perform the action
            time.sleep(0.5)
        else:
            print("Gripper planing failed")
    move_arm_to_pose(x=0.040, y=-0.00, z=0.336)
    set_gripper("Open")
    move_arm_to_pose(x=0.164, y=0.000,z=0.338)
    set_gripper("Close")
    move_arm_to_pose(x=0.040, y=-0.00, z=0.336)


    rclpy.shutdown() 

if __name__ == '__main__':
    main()

    






