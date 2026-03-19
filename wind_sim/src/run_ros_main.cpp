#include <algorithm>
#include <string>
#include <vector>
#include <cmath>
#include <random>

// Gazebo Transport
#include <gz/common/Profiler.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h> // 新增：用于发布 PlotJuggler 调试数据
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

// --- 全局变量 ---
gz::transport::Node node;
gz::transport::Node::Publisher pub_wind_cmd, pub_wind_end;
ros::Subscriber targetSub, howbig_sub, odom_sub;
ros::Publisher marker_pub, delay_odom_pub;
ros::Publisher pub_debug; // 新增：调试话题发布者

// --- 风场基础参数 (由话题/Rviz控制) ---
double base_wind_magnitude = 0.0;    // 基础风速
double base_wind_dir_yaw = 0.0;      // 基础风向角度 (弧度)
geometry_msgs::Quaternion wind_orientation; // Rviz显示用

// --- 波动参数配置 ---
const double REF_HEIGHT = 20.0;      // 基准高度
const double GUST_STD_DEV = 1.0;     // 阵风标准差 (m/s)
const double TURB_STD_DEV = 0.25;    // 风向扰动标准差 (rad)

// --- 随机数生成器 ---
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<> noise_mag(0.0, GUST_STD_DEV); 
std::normal_distribution<> noise_dir(0.0, TURB_STD_DEV); 

// --- 固定区域参数 ---
const double CENTER_X = 0.0;
const double CENTER_Y = 0.0;
const double CENTER_Z = 0.0;
const float RADIUS = 200.0;

// --- 无人机状态 ---
nav_msgs::Odometry::ConstPtr now_pose;
bool have_odom = false;

// --- 延迟里程计相关 ---
vector<nav_msgs::Odometry> odom_buffer;
ros::Time get_cmdtime;
nav_msgs::Odometry delay_odom;
bool have_delay_odom = false;


// --- 回调函数 ---

// 1. 设置基础风向
void goal_Cb(const geometry_msgs::PoseStamped::ConstPtr& msg){    
    base_wind_dir_yaw = tf::getYaw(msg->pose.orientation);
    wind_orientation = msg->pose.orientation;
    std::cout << "[Wind Set] Base Direction Yaw: " << base_wind_dir_yaw * 180.0 / M_PI << " deg" << std::endl;
}

// 2. 设置基础风速
void howbig_Cb(const std_msgs::Float64::ConstPtr& msg){
    base_wind_magnitude = msg->data;
    std::cout << "[Wind Set] Base Magnitude: " << base_wind_magnitude << " m/s" << std::endl;
}

// 3. 处理里程计
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    now_pose = msg;
    
    if (!have_odom) {
        have_odom = true;
        odom_buffer.emplace_back(*now_pose);
        get_cmdtime = ros::Time::now();
        wind_orientation.w = 1.0; 
    } else {
        odom_buffer.emplace_back(*now_pose);
        if ((ros::Time::now() - get_cmdtime).toSec() > 0.01) {
            delay_odom = odom_buffer[0];
            odom_buffer.erase(odom_buffer.begin());
            have_delay_odom = true;
            delay_odom_pub.publish(delay_odom);
        }
    }
}

void pub_delay_odom(const ros::TimerEvent& event) {
    if (have_delay_odom) delay_odom_pub.publish(delay_odom);
}

// 5. 主计算循环 (10Hz)
void timerCallback(const ros::TimerEvent& event) {
    if (!have_odom) return;

    // --- A. Rviz 可视化 (画球和箭头) ---
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_sphere;
    marker_sphere.header.frame_id = "map";
    marker_sphere.header.stamp = ros::Time::now();
    marker_sphere.ns = "wind_sphere";
    marker_sphere.id = 0;
    marker_sphere.type = visualization_msgs::Marker::SPHERE;
    marker_sphere.action = visualization_msgs::Marker::ADD;
    marker_sphere.pose.position.x = CENTER_X;
    marker_sphere.pose.position.y = CENTER_Y;
    marker_sphere.pose.position.z = CENTER_Z;
    marker_sphere.pose.orientation.w = 1.0;
    marker_sphere.scale.x = RADIUS * 2;
    marker_sphere.scale.y = RADIUS * 2;
    marker_sphere.scale.z = RADIUS * 2;
    marker_sphere.color.a = 0.2; 
    marker_sphere.color.r = 1.0;
    marker_array.markers.push_back(marker_sphere);

    visualization_msgs::Marker marker_arrow;
    marker_arrow.header.frame_id = "map";
    marker_arrow.header.stamp = ros::Time::now();
    marker_arrow.ns = "wind_arrow_base";
    marker_arrow.id = 1;
    marker_arrow.type = visualization_msgs::Marker::ARROW;
    marker_arrow.action = visualization_msgs::Marker::ADD;
    marker_arrow.pose.position.x = CENTER_X;
    marker_arrow.pose.position.y = CENTER_Y;
    marker_arrow.pose.position.z = CENTER_Z;
    marker_arrow.pose.orientation = wind_orientation; 
    marker_arrow.scale.x = 20.0; 
    marker_arrow.scale.y = 2.0;
    marker_arrow.scale.z = 2.0;
    marker_arrow.color.a = 1.0;
    marker_arrow.color.r = 0.0;
    marker_arrow.color.g = 1.0;
    marker_array.markers.push_back(marker_arrow);
    marker_pub.publish(marker_array);

    // --- B. 物理计算与发布 ---
    float drone_x = now_pose->pose.pose.position.x;
    float drone_y = now_pose->pose.pose.position.y;
    float drone_z = now_pose->pose.pose.position.z;

    float distance = sqrt(pow(drone_x - CENTER_X, 2) + 
                          pow(drone_y - CENTER_Y, 2) + 
                          pow(drone_z - CENTER_Z, 2));

    if (distance <= RADIUS) {
        // 1. 高度切变
        double shear_factor = std::max(0.0, (double)drone_z / REF_HEIGHT); 
        double shear_mag = base_wind_magnitude * shear_factor;

        // 2. 阵风
        double current_gust = noise_mag(gen);
        double final_mag = shear_mag + current_gust;
        if (final_mag < 0) final_mag = 0; 

        // 3. 湍流 (风向)
        double current_turb_yaw = noise_dir(gen);
        double final_yaw = base_wind_dir_yaw + current_turb_yaw;

        // 4. 发送给 Gazebo (Vector3d)
        double final_wind_x = cos(final_yaw);
        double final_wind_y = sin(final_yaw);

        gz::msgs::Vector3d msg_wind;
        msg_wind.set_x(final_wind_x);
        msg_wind.set_y(final_wind_y);
        msg_wind.set_z(final_mag);
        pub_wind_cmd.Publish(msg_wind);

        // --- C. 发送给 PlotJuggler (ROS Topic) ---
        geometry_msgs::Vector3 debug_msg;
        
        // 转换弧度到角度 (0-360)
        double deg = final_yaw * 180.0 / M_PI;
        // 规范化到 0-360
        while (deg < 0) deg += 360.0;
        while (deg >= 360.0) deg -= 360.0;

        debug_msg.x = deg;       // 风向 (0-360度)
        debug_msg.y = final_mag; // 风速 (m/s)
        debug_msg.z = 0;
        
        pub_debug.publish(debug_msg);

    } else {
        // 区域外停止风
        gz::msgs::Vector3d flag; 
        pub_wind_end.Publish(flag);

        // 区域外 Debug 显示为 0
        geometry_msgs::Vector3 debug_msg;
        debug_msg.x = 0;
        debug_msg.y = 0;
        debug_msg.z = 0;
        pub_debug.publish(debug_msg);
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "realtime_wind_fluctuation");
    ros::NodeHandle nh;

    // Gazebo 话题
    pub_wind_cmd = node.Advertise<gz::msgs::Vector3d>("/direction");
    pub_wind_end = node.Advertise<gz::msgs::Vector3d>("/end_wind");

    // ROS 订阅
    targetSub = nh.subscribe("/goal", 1, &goal_Cb, ros::TransportHints().tcpNoDelay()); 
    howbig_sub = nh.subscribe("/howbig", 1, &howbig_Cb, ros::TransportHints().tcpNoDelay()); 
    odom_sub = nh.subscribe("/mavros/local_position/odom", 1, &odom_cb, ros::TransportHints().tcpNoDelay()); 
    
    // ROS 发布
    delay_odom_pub = nh.advertise<nav_msgs::Odometry>("/delay_odom", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/wind_Area", 10);
    
    // !!! 新增：发布给 PlotJuggler 的调试话题 !!!
    pub_debug = nh.advertise<geometry_msgs::Vector3>("/wind_debug", 10);

    // 定时器
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
    ros::Timer timer2 = nh.createTimer(ros::Duration(0.001), pub_delay_odom);

    std::cout << "--- Realtime Wind with PlotJuggler Debug Started ---" << std::endl;
    std::cout << "Debug Topic: /wind_debug (x: deg, y: m/s)" << std::endl;

    ros::spin();
    return 0;
}
