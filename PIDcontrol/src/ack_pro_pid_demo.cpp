#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <thread>
#include "dynamic_reconfigure/server.h"
#include <unistd.h>
#include <sys/types.h>
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
using namespace std;
// #include "/home/racecar/racecar_ws/devel/include/racecar/dr_2Config.h"
ofstream in;
int Terminal_v; //速度跟随器PID参数
double kp, ki, kd,brake,goalRadius,cmd_vel_num,wheelbase2,cmd_vel_min,cmd_vel_angular_front,file,cmd_vel_line_front,goal_ditance,goal_ditance_d,ang_num;
bool goal_reached,goal_received,goal_bool;
typedef struct //增量式速度PID结构体
{
    float kp;
    float ki;
    float kd;
    float error_now;
    float error_l;
    float error_ll;
    float pid_out;
} Pid_speed;

Pid_speed pid_speed;

class ack_pro
{
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_odometry,goal_sub,quick_vel,sub_vel,sub_encoder;
    ros::Publisher pub_vel,marker_pub,goal_pub;
    ros::Timer timer2;
    tf::TransformListener tf_listener;
    visualization_msgs::Marker points, line_strip, goal_circle;
    geometry_msgs::Twist cmd_vel,pub_cmd_vel; //小车接收到的速度对象
    geometry_msgs::Point odom_goal_pos;
    geometry_msgs::PoseStamped car_pose;
    nav_msgs::Odometry odom;
    std_msgs::Int8 goal_num;
    double car_vel;
    float wheelbase,speed_error,speed_error_front;
    int vel_max,vel_max_v,vel_min,z_zhong;
    void goalReachingCB(const ros::TimerEvent &);
    void Get_vel(const geometry_msgs::Twist::ConstPtr &velMsg);
    void Get_Odometry(const nav_msgs::Odometry::ConstPtr &odomMsg);
    void Get_encoder(const nav_msgs::Odometry::ConstPtr &encoder_vel);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg);
    void Get_quick(const std_msgs::String::ConstPtr& msg_quick);
public:
    ack_pro();
    void initMarker();
    ~ack_pro();
    int Get_pidout(float error);
    float convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase);
    float dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2);
    double getCar2GoalDist();
};
ack_pro::ack_pro()
{
    ROS_INFO("小车控制节点开启");
    ros::NodeHandle pn("~");
    /*PD参数初始化*/
    pn.param("kp", kp, 2.0);
    pn.param("ki", ki, 0.0);
    pn.param("kd", kd, 0.0);
    pn.param("cmd_vel_num", cmd_vel_num, 1.2);
    pn.param("ang_num", ang_num, 1.2);
    pn.param("brake", brake, -1.2);
    pn.param("vel_max", vel_max, 1800);
    pn.param("vel_min", vel_min, 1500);
    pn.param("z_zhong", z_zhong, 92);
    pn.param("goalRadius", goalRadius, 1.0);
    pn.param("goal_ditance", goal_ditance, 1.0);
    pn.param("goal_ditance_d", goal_ditance_d, 0.5);
    pn.param("Terminal_v", Terminal_v, 1590);
    pn.param("wheelbase2", wheelbase2, 0.3);
    pn.param("cmd_vel_min", cmd_vel_min, 0.5);
    pn.param("file", file, 1.0);
    ROS_INFO("cmd_vel_num---%0.2f",cmd_vel_num);
    ROS_INFO("ang_num---%0.2f",ang_num);
    ROS_INFO("brake---%0.2f",brake);
    ROS_INFO("vel_max---%d",vel_max);
    ROS_INFO("vel_min---%d",vel_min);
    ROS_INFO("z_zhong---%d",z_zhong);
    ROS_INFO("goalRadius---%0.2f",goalRadius);
    ROS_INFO("goal_ditance---%0.2f",goal_ditance);
    ROS_INFO("goal_ditance_d---%0.2f",goal_ditance_d);
    ROS_INFO("Terminal_v---%d",Terminal_v);
    ROS_INFO("wheelbase2---%0.2f",wheelbase2);
    ROS_INFO("cmd_vel_min---%0.2f",cmd_vel_min);
    ROS_INFO("file---%0.2f",file);
    in.open("/home/racecar/log/P"+to_string((float)round((kp)*10)/10)+"_"+"I"+to_string((float)round((ki)*10)/10)+"_"+"D"+to_string((float)round((kd)*10)/10)+"_"+to_string((float)round((file)*10)/10)+".txt",ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
    pid_speed.kp = kp;
    pid_speed.ki = ki;
    pid_speed.kd = kd;
    ROS_INFO("kp---%0.2f",pid_speed.kp);
    ROS_INFO("ki---%0.2f",pid_speed.ki);
    ROS_INFO("kd---%0.2f",pid_speed.kd);
    pid_speed.error_l=0;
    pid_speed.error_ll=0;
    /* 订阅/发布初始化 */
    wheelbase = float(wheelbase2);
    ROS_INFO("wheelbase---%0.2f==",wheelbase);
    pub_cmd_vel.angular.z = z_zhong;
    pub_cmd_vel.linear.x = vel_min;
    vel_max_v=vel_max;
     //订阅编码器
    sub_encoder = n_.subscribe<nav_msgs::Odometry>("/encoder", 1, &ack_pro::Get_encoder, this);
    sub_odometry = n_.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, &ack_pro::Get_Odometry, this);
    sub_vel = n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &ack_pro::Get_vel, this);
    quick_vel = n_.subscribe<std_msgs::String>("/quick", 1, &ack_pro::Get_quick, this);
    pub_vel = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1); //发布速度、角速度
    goal_pub = n_.advertise<std_msgs::Int8>("go_loop" , 1);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 1); //发布路径到rviz
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &ack_pro::goalCB, this); //订阅movebase目标点
    timer2 = n_.createTimer(ros::Duration((0.5) / 10), &ack_pro::goalReachingCB, this);
    goal_num.data=1;
    goal_reached =true;
    goal_received = false;
    initMarker();
}
// rviz可视化初始化
void ack_pro::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    points.scale.x = 0.07;
    points.scale.y = 0.07;
    line_strip.scale.x = 0.02;
    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}

ack_pro::~ack_pro()
{
    pub_cmd_vel.linear.x = 1500;
    pub_cmd_vel.angular.z = z_zhong;
    pub_vel.publish(pub_cmd_vel);
    in.close();//关闭文件
    printf("文件关闭\n");
    printf("小车控制节点关闭\n");
}
void ack_pro::Get_quick(const std_msgs::String::ConstPtr& msg_quick){
    cmd_vel_num=std::stod(msg_quick->data.c_str());
    ROS_INFO("<%0.2f>倍速", cmd_vel_num);
}
void ack_pro::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goalMsg)
{ //目标点回调函数
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0), *goalMsg, "map", odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
        goal_received = true;
        ROS_INFO("<<<<<<<<<<<<-----新的目标点----->>>>>>>>>>>>>");
        goal_bool=true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}
double ack_pro::getCar2GoalDist()
{ //计算车到目标点的距离
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;
    double dist2goal = sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
    return dist2goal;
}
void ack_pro::goalReachingCB(const ros::TimerEvent &)
{ //小车到达目标点后
    if(goal_received){
        double car2goal_dist = getCar2GoalDist();
        if (car2goal_dist < goal_ditance and goal_bool==true){
            goal_pub.publish(goal_num);
            ROS_INFO("发送第<%0.2d>的目标点",goal_num.data);
            goal_num.data++;
            goal_bool=false;
        }
        if (car2goal_dist < (goal_ditance+goal_ditance_d)){
            vel_max=Terminal_v;
        }else{
            vel_max=vel_max_v;
        }
        ROS_INFO("目标点的距离：%0.2f",car2goal_dist);
        if (car2goal_dist < goalRadius)
        {
            goal_reached = false;
            goal_received = false;
            pub_cmd_vel.linear.x = 1500.0;
            pub_cmd_vel.angular.z = z_zhong;
            pub_vel.publish(pub_cmd_vel);
            ROS_INFO("抵达目标 !!!!!!!");
        }else{
            goal_reached = true;
        }
    }  
}
int ack_pro::Get_pidout(float error)
{
    float p_out, i_out, d_out;
    pid_speed.error_now = error;
    // if (error>0){
    //     p_out=(0.5)* pid_speed.kp * (pid_speed.error_now - pid_speed.error_l);;
    // }else{
    //     p_out = pid_speed.kp * (pid_speed.error_now - pid_speed.error_l);
    // }
    p_out = pid_speed.kp * (pid_speed.error_now - pid_speed.error_l);
    i_out = pid_speed.ki * pid_speed.error_now;
    ROS_INFO("i_out-----%0.2f",i_out);
    d_out = pid_speed.kd * (pid_speed.error_now - 2 * pid_speed.error_l + pid_speed.error_ll);
    pid_speed.pid_out = p_out + i_out + d_out;
    ROS_INFO("error=%0.2f p_out=%0.2f i_out=%0.2f d_out=%0.2f pid_speed.pid_out=%0.2f",error,p_out,i_out,d_out,pid_speed.pid_out);
    pid_speed.error_ll=pid_speed.error_l;
    pid_speed.error_l=pid_speed.error_now;
    return pid_speed.pid_out;
}
float ack_pro::convert_trans_rot_vel_to_steering_angle(float v, float omega, float wheelbase)
{
    float radius;
    if (omega == 0 || v == 0)
    {
        return 0;
    }
    radius = v / omega;
    return atan(wheelbase / radius);
}
void ack_pro::Get_Odometry(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    odom = *odomMsg;
}
void ack_pro::Get_encoder(const nav_msgs::Odometry::ConstPtr &encoder_vel)
{
    //获取小车速度
    car_vel = encoder_vel->twist.twist.linear.x;
}

void ack_pro::Get_vel(const geometry_msgs::Twist::ConstPtr &velMsg)
{
    if (goal_reached){
        cmd_vel.angular.z = velMsg->angular.z;
        cmd_vel.linear.x = velMsg->linear.x * cmd_vel_num;
        if (cmd_vel.angular.z == 0 and cmd_vel.linear.x == 0) {
            speed_error = cmd_vel.linear.x - car_vel;
            pub_cmd_vel.linear.x = pub_cmd_vel.linear.x+Get_pidout(speed_error);
            if (pub_cmd_vel.linear.x > vel_max)
                pub_cmd_vel.linear.x = vel_max;
            else if (pub_cmd_vel.linear.x < vel_min)
                pub_cmd_vel.linear.x = vel_min;
            pub_cmd_vel.angular.z = cmd_vel_angular_front;
            pub_vel.publish(pub_cmd_vel);
        }else{
            if (cmd_vel.linear.x<0){
                cmd_vel.linear.x=cmd_vel_min;
            }
            pub_cmd_vel.angular.z = int(ang_num*(convert_trans_rot_vel_to_steering_angle(cmd_vel.linear.x, cmd_vel.angular.z, wheelbase) * 180 / 3.1416) + z_zhong);
            // if(abs(pub_cmd_vel.angular.z-z_zhong)>5){
            //     pub_cmd_vel.angular.z=(pub_cmd_vel.angular.z-z_zhong)*ang_num+z_zhong;
            // }
            if(abs(pub_cmd_vel.angular.z-z_zhong)>56 and cmd_vel.linear.x>1.5){
                cmd_vel.linear.x=1.5;
            }
            speed_error = cmd_vel.linear.x - car_vel;
            if (pub_cmd_vel.angular.z > 150)
                pub_cmd_vel.angular.z = 150;
            else if (pub_cmd_vel.angular.z < 30)
                pub_cmd_vel.angular.z = 30;
            ROS_INFO("======================");
            pub_cmd_vel.linear.x = pub_cmd_vel.linear.x+Get_pidout(speed_error) ;
            if (pub_cmd_vel.linear.x < 0)
            {
                pub_cmd_vel.linear.x = 1500;
            }
            if (pub_cmd_vel.linear.x > vel_max)
                pub_cmd_vel.linear.x = vel_max;
            else if (pub_cmd_vel.linear.x < vel_min)
                pub_cmd_vel.linear.x = vel_min;
            
            if (speed_error < brake) //点刹
            {
                ROS_INFO("------->刹车<------");
                pub_cmd_vel.linear.x = 1500;
            }
            pub_vel.publish(pub_cmd_vel);
            in<<(double)round((cmd_vel.linear.x)*10)/10<<"-----"<<(double)round((car_vel)*10)/10<<"\n";
        }
        cmd_vel_angular_front = pub_cmd_vel.angular.z;
        cmd_vel_line_front = pub_cmd_vel.linear.x;
        speed_error_front = speed_error;
        ROS_INFO("目标速度=%0.2f 目标角度=%0.2f 实际速度=%0.2f 输出速度=%f 输出角度=%f", cmd_vel.linear.x,cmd_vel.angular.z,car_vel,pub_cmd_vel.linear.x,pub_cmd_vel.angular.z);
        ROS_INFO("=======================");
    }
}
float ack_pro::dis_point(const geometry_msgs::PoseStamped &x1, const geometry_msgs::PoseStamped &x2)
{
    float dis;
    dis = sqrt(pow(x1.pose.position.x - x2.pose.position.x, 2) + pow(x1.pose.position.y - x2.pose.position.y, 2));
    // ROS_INFO("小车相对于目标点的距离为:(%.2f)", dis);
    return dis;
}
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ack_pro");
    ack_pro controller;
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    ros::spin();
    return 0;
}
