#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "first_project/Odom.h"
#include "first_project/reset_odom.h"

class OdomNode
{
public:
    OdomNode() {
        last_time = ros::Time::now();
        n.getParam("/length", length);
        n.getParam("/x", last_x);
        n.getParam("/y", last_y);
        n.getParam("/th", last_theta);

        sub = n.subscribe("/speed_steer", 1, &OdomNode::computeOdometry, this);
        pub = n.advertise<nav_msgs::Odometry>("/odometry", 1);
        pub1 = n.advertise<first_project::Odom>("/custom_odometry", 1);
        reset_odom_service = n.advertiseService("/reset_odom", &OdomNode::reset_odom, this);
    }

    // This is the callback function that will be called when a new message is received, it compute the odometry taking into account the speed and the steering angle (msg->x and msg->y) using Runge-Kutta 2th order method and call the publishing functions
    void computeOdometry(const geometry_msgs::Quaternion::ConstPtr& msg) {
        // Get current time
        ros::Time current_time = ros::Time::now();

        // Compute dt
        float dt = (current_time - last_time).toSec();

        // compute vf and w
        float vf = msg->x / cos(msg->y);
        float w = vf * tan(msg->y) / length;

        // Compute current position
        float x = last_x + dt * vf * cos(last_theta + (w * dt) / 2);
        float y = last_y + dt * vf * sin(last_theta + (w * dt) / 2);
        float theta = last_theta + dt * w;

        // Compute odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg = createOdometryMsg(x, y, theta, current_time);

        // Compute custom message
        first_project::Odom custom_msg;
        custom_msg = createCustomMsg(x, y, theta, current_time);

        // Compute TF
        tf::Transform transform = createTF(x, y, theta);
        publishTF(transform, current_time);

        // Publish odometry, custom messages and TF
        publishOdometryMsg(odom_msg);
        publishCustomMsg(custom_msg);

        // Update last values
        last_time = current_time;
        last_x = x;
        last_y = y;
        last_theta = theta;
    }

    // Reset service
    bool reset_odom(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res) {
        last_time = ros::Time::now();
        last_x = 0;
        last_y = 0;
        last_theta = 0;
        return true;
    }

private:
    nav_msgs::Odometry createOdometryMsg(float x, float y, float theta, ros::Time time) {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        tf::Quaternion q;
        q.setRPY(0, 0, theta);

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        return odom_msg;
    }

    first_project::Odom createCustomMsg(float x, float y, float theta, ros::Time time) {
        first_project::Odom custom_msg;
        custom_msg.x = x;
        custom_msg.y = y;
        custom_msg.th = theta;
        custom_msg.timestamp = time.toSec();

        return custom_msg;
    }

    tf::Transform createTF(float x, float y, float theta) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);
        return transform;
    }

    void publishOdometryMsg(nav_msgs::Odometry msg) {
        pub.publish(msg);
    }

    void publishCustomMsg(first_project::Odom msg1) {
        pub1.publish(msg1);
    }

    void publishTF(tf::Transform transform, ros::Time time) {
        odom_broadcaster.sendTransform(tf::StampedTransform(transform, time, "world", "base_link"));
    }

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub1;
    ros::ServiceServer reset_odom_service;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time last_time;
    float last_x;
    float last_y;
    float last_theta;

    float length;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_node");
    OdomNode odom_node;
    ros::spin();
    return 0;
}