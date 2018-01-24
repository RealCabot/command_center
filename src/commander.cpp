#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

#include <nav_msgs/GetPlan.h>

using namespace move_base_msgs;

struct Pose {
    double x;
    double y;
    double angle;
};

const Pose GOAL = {2, -22, -90};

typedef actionlib::SimpleActionClient<MoveBaseAction> MoveBaseClient;
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const MoveBaseResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO_STREAM("Got some feedback. Too lazy to print them");
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "commander");
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
     }

    ROS_INFO("move_base started, sending goal.");

    MoveBaseGoal goal;
    geometry_msgs::Quaternion goal_quaternion = tf::createQuaternionMsgFromYaw(angles::from_degrees(GOAL.angle));
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = GOAL.x;
    goal.target_pose.pose.position.y = GOAL.y;
    goal.target_pose.pose.orientation.x = goal_quaternion.x;
    goal.target_pose.pose.orientation.y = goal_quaternion.y;
    goal.target_pose.pose.orientation.z = goal_quaternion.z;
    goal.target_pose.pose.orientation.w = goal_quaternion.w;

    // Chris' service 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient <nav_msgs::GetPlan> ("/move_base/make_plan");
    nav_msgs::GetPlan srv;

    ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("plan", 1000);

    // GetPlan -> start
    srv.request.start.pose.position.x = 0; // start -> PoseStamped 
    srv.request.start.pose.position.y = 0;
    srv.request.start.pose.position.z = 0;
    srv.request.start.pose.orientation.x = 0; 
    srv.request.start.pose.orientation.y = 0;
    srv.request.start.pose.orientation.z = 0;
    srv.request.start.pose.orientation.w = 0;
    srv.request.start.header.frame_id = "map";
    srv.request.start.header.stamp = ros::Time::now();
    
    // GetPlan -> goal
    srv.request.goal = goal.target_pose; // goal -> PoseStamped

    // GetPlan -> tolerance
    srv.request.tolerance = 0.5;


    if (client.call(srv)) {
        plan_pub.publish(srv.response.plan);
    }
    else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    ac.sendGoal(goal);

    /*if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
*/
    return 0;
}