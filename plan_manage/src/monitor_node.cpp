#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <future>
#include <thread>
#include <Eigen/Eigen>
#include <quadrotor_msgs/GoalSet.h>
using namespace Eigen;
ros::Timer cmd_timer;
ros::Publisher waypointPub_;
ros::Time heartbeat_time_(0);
bool flag_pub_record_point = false;
bool flag_have_recived_waypoint_ = false;
Vector3d record_wp_(0, 0, 0); 
void restart_process(const std::string &command)
{
    std::cout << "restart process: " << command << std::endl;
    system((command + " &").c_str());
    std::cout << "restarted: " << command << std::endl;
}
void kill_process(const std::string &process_name)
{
    std::cout << "Killing process: " << process_name << std::endl;
    system(("pkill -9 " + process_name).c_str());
    std::cout << "Process killed: " << process_name << std::endl;
}
void heartbeatCallback(std_msgs::EmptyPtr msg)
{
    heartbeat_time_ = ros::Time::now();
}
void monitorCallback(const ros::TimerEvent &e)
{
    if (heartbeat_time_.toSec() <= 1e-5)
    {
        return;
    }
    static int count = 0;
    static bool flag_pub_switch_point = true;
    ros::Time time_now = ros::Time::now();
    if ((time_now - heartbeat_time_).toSec() > 4 && flag_pub_switch_point)
    {
        flag_pub_switch_point = false;
        cmd_timer.stop();
        ROS_ERROR("[monitor] Lost heartbeat from the planner, planner maybe dead !!! will restart diff_planner !!!");
        std::string process_name = "diff_planner";                                                          
        std::string start_command = "roslaunch diff_planner single_drone_interactive.launch"; 
        std::future<void> kill_future = std::async(std::launch::async, kill_process, process_name);
        kill_future.wait();
        std::future<void> start_future = std::async(std::launch::async, restart_process, start_command);
        start_future.wait();
        ROS_INFO("cmd timer start");
        flag_pub_record_point = true;
        cmd_timer.start();
    }
    if (flag_pub_record_point)
    {
        count++;
        if (count == 3)
        {
            count = 0;  
            if (flag_have_recived_waypoint_)
            {
                quadrotor_msgs::GoalSet msg;
                msg.goal[0] = record_wp_(0); 
                msg.goal[1] = record_wp_(1);
                msg.goal[2] = record_wp_(2);
                waypointPub_.publish(msg); 
                flag_pub_record_point = false;
                flag_have_recived_waypoint_ = false;          
                ROS_ERROR("publish record point msg: %f, %f, %f", msg.goal[0], msg.goal[1], msg.goal[2]);
                flag_pub_switch_point = true;
            }
        }
    }
}
  void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->goal[2] < -0.1 || msg->goal[1] > 20000 || msg->goal[0] > 20000) 
      return;
    ROS_INFO("Monitor received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);
    record_wp_(0)=msg->goal[0];
    record_wp_(1)=msg->goal[1];
    record_wp_(2)=msg->goal[2];
    flag_have_recived_waypoint_ = true;
  }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor_node");
    ros::NodeHandle nh("~");
    ros::Subscriber heartbeat_sub = nh.subscribe("/drone_0_traj_server/heartbeat", 10, heartbeatCallback);
    ros::Subscriber waypoint_sub_ = nh.subscribe("/goal_with_id", 1, waypointCallback);
    waypointPub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
    ros::Timer monitor_timer = nh.createTimer(ros::Duration(1.0), monitorCallback);
    ros::Duration(1.0).sleep();
    ROS_INFO("[monitor:] monitor node is ready.");
    ros::spin();
    return 0;
}