#include <ros/ros.h>
#include <yaml-cpp/yaml.h>  //YAML开源库yaml-cpp
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>

using namespace std;

ros::Publisher point_pub;
ros::Publisher yaw_pub;
ros::Publisher takeoff_land_pub;
ros::Publisher backcommand_pub;
ros::Publisher startcommand_pub;
ros::Subscriber odom_sub;
ros::Subscriber startcommand_sub;
ros::Subscriber backcommand_sub;
ros::Subscriber rc_sub;
ros::Timer timer;

enum RC_EIGHT_STATE
{
    RC_EIGHT_UP = 999,
    RC_EIGHT_MIDDLE = 1499,
    RC_EIGHT_DOWN = 1999
};

RC_EIGHT_STATE rc_eight_pre = RC_EIGHT_DOWN;
bool rc_init = true;

// 定义结构体表示每个点的数据
struct pytStr {
    double x;
    double y;
    double z;
    double yaw = -100;
    double time;
};

enum FLIGT_TYPE{
    PP = 1,
    PP_TIME = 2,
    PP_YAW = 3,
    PP_YAW_TIME = 4
};

Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
Eigen::Vector3d back_pos{0.0, 0.0, 1.2};
geometry_msgs::PoseStamped goal;  //目标点
vector<pytStr> pytVector;  //存放yaml读取的点
vector<pytStr> start_pytVector, back_pytVector;  //存放yaml读取的点
int counts = 0;  //正在规划的点位次序
double distance_ = 10.0;  //规划的点到当前点的距离
double next_distance; 
quadrotor_msgs::PositionCommand cmd_yaw;
bool time_flag = false;
int fligt_type;
bool trigger = false;
int flag_start_plan;
int flag_back_plan;

void readpyt(string file_path)
{
    if(file_path.empty())
    {
        ROS_ERROR("The YAML file path is empty,Failed to load pyt!");
        return;
    }
    // 读取YAML文件
    YAML::Node pyt_yaml = YAML::LoadFile(file_path);   
    YAML::Node pyt_type;
    YAML::Node pyt_type_;

    //验证YAML文件是不是为空
    if (pyt_yaml.IsNull())
    {
        ROS_ERROR("The YAML file is empty,Failed to load pyt!");
        return;  
    }

    int size_arr;
    if(fligt_type == FLIGT_TYPE::PP){
        pyt_type = pyt_yaml["test1"];
        size_arr = 3;
    }
    else if(fligt_type == FLIGT_TYPE::PP_YAW || fligt_type == FLIGT_TYPE::PP_TIME){
        if(fligt_type == FLIGT_TYPE::PP_YAW){
            pyt_type = pyt_yaml["test2"];
        }
        else{
            pyt_type = pyt_yaml["test3"];
        }
        size_arr = 4;
    }
    else{
        pyt_type = pyt_yaml["test4"];
        size_arr = 5;
    }

    pyt_type_ = pyt_yaml["test_back"];
    
    // 验证每个数组是否符合该模式的元素个数
    for (size_t i = 0; i < pyt_type.size(); i++)
    {
        if (pyt_type[i].size() != size_arr)
        {
            ROS_ERROR("Invalid point format at index %zu. Each point should contain three elements.", i);
            return;  
        }
    }

    for (size_t i = 0; i < pyt_type_.size(); i++)
    {
        if (pyt_type_[i].size() != 3)
        {
            ROS_ERROR("Invalid back_point format at index %zu. Each point should contain three elements.", i);
            return;  
        }
    }

    // 将读取的点位添加到vector中
    for (size_t i = 0; i < pyt_type.size(); i++)
    {
        pytStr pyt_;
        pyt_.x = pyt_type[i][0].as<double>();
        pyt_.y = pyt_type[i][1].as<double>();
        pyt_.z = pyt_type[i][2].as<double>();   //YAML的元素转换为double
        if(fligt_type == FLIGT_TYPE::PP_YAW){
            pyt_.yaw = pyt_type[i][3].as<double>();
        }
        else if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){
            pyt_.yaw = pyt_type[i][3].as<double>();
            pyt_.time = pyt_type[i][4].as<double>();
            if(pyt_.time < 0){
                ROS_ERROR("第%d个数组传入错误的等待时间", i+1);
                return;
            }
        }
        else if(fligt_type == FLIGT_TYPE::PP_TIME){
            pyt_.time = pyt_type[i][3].as<double>();
             if(pyt_.time < 0){
                ROS_ERROR("第%d个数组传入错误的等待时间", i+1);
                return;
            }
        }
        start_pytVector.push_back(pyt_);
    }

        // 将读取的点位添加到back_vector中
    for (size_t i = 0; i < pyt_type_.size(); i++)
    {
        pytStr pyt_;
        pyt_.x = pyt_type_[i][0].as<double>();
        pyt_.y = pyt_type_[i][1].as<double>();
        pyt_.z = pyt_type_[i][2].as<double>();   //YAML的元素转换为double 
        back_pytVector.push_back(pyt_);
    }      

    ROS_INFO("Loaded pyt from YAML file:");
    for (size_t i = 0; i < start_pytVector.size(); i++)
    {
        ROS_INFO("pyt %d: [%f, %f, %f, %f, %f]", i+1, start_pytVector[i].x, start_pytVector[i].y, start_pytVector[i].z, start_pytVector[i].yaw, start_pytVector[i].time);
    }
    for (size_t i = 0; i < back_pytVector.size(); i++)
    {
        ROS_INFO("back_pyt %d: [%f, %f, %f]", i+1, back_pytVector[i].x, back_pytVector[i].y, back_pytVector[i].z);
    }
    cout << "距离判断" << next_distance <<  endl;

    return;
}

void odom_goal_cb(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;
}

void Point_send(const ros::TimerEvent& event) 
{
    if(!trigger){
        return;
    }
    else if(pytVector.size() == 0)   //判断是否还有点
    {
        trigger = false;
        return;
    }

    if(counts == 0){
        //先发送第一个目标点
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = pytVector[counts].x;
        goal.pose.position.y = pytVector[counts].y;
        goal.pose.position.z = pytVector[counts].z;
        point_pub.publish(goal);   
        if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){   
            ROS_INFO("Publish the first pyt: [x:%f, y:%f, z:%f, yaw:%f, time:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, cmd_yaw.yaw, pytVector[counts].time);
        }
        else if(fligt_type == FLIGT_TYPE::PP_TIME){
            ROS_INFO("Publish the first pyt: [x:%f, y:%f, z:%f, time:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pytVector[counts].time);
        }
        else if(fligt_type == FLIGT_TYPE::PP_YAW){
            ROS_INFO("Publish the first pyt: [x:%f, y:%f, z:%f, yaw:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pytVector[counts].yaw);
        }
        else{
            ROS_INFO("Publish the first pyt: [x:%f, y:%f, z:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        }
        counts++;
    }

    int counts_pre = counts - 1;
    Eigen::Vector3d point_cur{pytVector[counts_pre].x, pytVector[counts_pre].y, pytVector[counts_pre].z};
    distance_ = (point_cur - odom_pos_).norm();
    // ROS_INFO("Distance to next point: %f", distance_);

    if (distance_ < next_distance && counts < pytVector.size())   //到达目标点并且还有下一个点
    {
        if(fligt_type == FLIGT_TYPE::PP_TIME || fligt_type == FLIGT_TYPE::PP_YAW_TIME){
            timer.stop();
            ROS_INFO("Timer stopped.");

            double time_wait = pytVector[counts_pre].time;   //等待时间
            ros::Duration(time_wait).sleep();

            timer.start();
            ROS_INFO("Timer started.");
        }

        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = pytVector[counts].x;
        goal.pose.position.y = pytVector[counts].y;
        goal.pose.position.z = pytVector[counts].z;
        point_pub.publish(goal);   
        if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){   
            ROS_INFO("Publish the next pyt: [x:%f, y:%f, z:%f, yaw:%f, time:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, cmd_yaw.yaw, pytVector[counts].time);
        }
        else if(fligt_type == FLIGT_TYPE::PP_TIME){
            ROS_INFO("Publish the next pyt: [x:%f, y:%f, z:%f, time:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pytVector[counts].time);
        }
        else if(fligt_type == FLIGT_TYPE::PP_YAW){
            ROS_INFO("Publish the next pyt: [x:%f, y:%f, z:%f, yaw:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, cmd_yaw.yaw);
        }
        else{
            ROS_INFO("Publish the next pyt: [x:%f, y:%f, z:%f]", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        }
        counts++;
    }

    if (distance_ > next_distance && counts_pre < pytVector.size())
    {
        if(fligt_type == FLIGT_TYPE::PP_YAW || fligt_type == FLIGT_TYPE::PP_YAW_TIME) 
        {   
            cmd_yaw.yaw = pytVector[counts_pre].yaw;
            yaw_pub.publish(cmd_yaw);
        }
    }

}

void startplan_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(start_pytVector.size() == 0)
    {
        ROS_ERROR("No pyt loaded!");
        return;
    }

    pytVector.assign(start_pytVector.begin(), start_pytVector.end());
    counts = 0;    
    trigger = true;
    ROS_INFO("Get start trigger.");
}

void backplan_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(back_pytVector.size() == 0)
    {
        ROS_ERROR("No pyt loaded!");
        return;
    }

    pytVector.clear();
    pytVector.assign(back_pytVector.begin(), back_pytVector.end());
    counts = 0;    
    fligt_type = FLIGT_TYPE::PP;
    trigger = true;
    ROS_INFO("Get back trigger.");
}

void rc_cb(mavros_msgs::RCInConstPtr pMsg)
{
    uint16_t rc_eight_cur = pMsg->channels[7]; // 获取遥控器八通道的值

    if (rc_eight_cur != RC_EIGHT_DOWN && rc_init){
        // 如果不是 RC_EIGHT_DOWN，则返回，不执行后续逻辑
        return;
    }
    else{
        rc_init = false;
    }

    // RC_EIGHT_DOWN 转化成 RC_EIGHT_MIDDLE 状态时发布 takeoff 话题
    if (rc_eight_cur == RC_EIGHT_MIDDLE && rc_eight_pre == RC_EIGHT_DOWN)
    {
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 1;
        takeoff_land_pub.publish(takeoff_msg);
        cout << "down --> middle" << endl;
        rc_eight_pre = RC_EIGHT_MIDDLE; 
        return;
    }

    // RC_EIGHT_MIDDLE 转化成 RC_EIGHT_UP 状态时发布 move 话题
    if (rc_eight_cur == RC_EIGHT_UP && rc_eight_pre == RC_EIGHT_MIDDLE)
    {
        geometry_msgs::PoseStamped startcommand_msg;
        startcommand_msg.header.stamp = ros::Time::now();
        startcommand_pub.publish(startcommand_msg);
        cout << "middle --> up" << endl;
        rc_eight_pre = RC_EIGHT_UP; 
        return;
    }

    // RC_EIGHT_UP 转化成 RC_EIGHT_MIDDLE 状态时发布 back 话题
    if (rc_eight_cur == RC_EIGHT_MIDDLE && rc_eight_pre == RC_EIGHT_UP)
    {
        geometry_msgs::PoseStamped backtrigger_msg;
        backtrigger_msg.header.stamp = ros::Time::now();
        backcommand_pub.publish(backtrigger_msg);
        cout << "up --> middle" << endl;
        rc_eight_pre = RC_EIGHT_MIDDLE; 
        return;
    }

    // RC_EIGHT_MIDDLE 转化成 RC_EIGHT_DOWN 状态时发布 land 话题
    if (rc_eight_cur == RC_EIGHT_DOWN && rc_eight_pre == RC_EIGHT_MIDDLE)
    {
        quadrotor_msgs::TakeoffLand takeoff_msg;
        takeoff_msg.takeoff_land_cmd = 2;
        takeoff_land_pub.publish(takeoff_msg);
        cout << "middle --> down" << endl;
        rc_eight_pre = RC_EIGHT_DOWN; 
        return;
    }
}
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "multipointplan_node");
    ros::NodeHandle nh;

    string yaml_path;
    if (!nh.getParam("/multipointplan/yaml_path", yaml_path) || 
        !nh.getParam("/multipointplan/next_distance", next_distance) || 
        !nh.getParam("/multipointplan/start_plan", flag_start_plan) ||
        !nh.getParam("/multipointplan/back_plan", flag_back_plan) ||
        !nh.getParam("/multipointplan/fligt_type", fligt_type)) 
    {
        ROS_ERROR("Failed to get parameter ,please check it");
        return 1;
    }

    switch (fligt_type) {
        case 1:
        case 2:
        case 3:
        case 4:
            break;
        default:
            ROS_ERROR("Failed to get a right fligt_type, please check it");
            break;
    }

    readpyt(yaml_path);

    odom_sub = nh.subscribe("odom_topic", 10, odom_goal_cb);
    if(flag_start_plan){
        startcommand_sub = nh.subscribe("/move_base_simple/goal", 10, startplan_cb);
    }
    if(flag_back_plan){
        backcommand_sub = nh.subscribe("/back_trigger", 10, backplan_cb);
    }
    rc_sub = nh.subscribe("/mavros/rc/in", 10, rc_cb);
    takeoff_land_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
    startcommand_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    backcommand_pub = nh.advertise<geometry_msgs::PoseStamped>("/back_trigger", 10);
    point_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);
    yaw_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/yaw", 10);
    timer = nh.createTimer(ros::Duration(0.01), Point_send);
    
    ros::spin();
}
   