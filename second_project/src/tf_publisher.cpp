#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/LinearMath/Quaternion.h"
#include "time.h"

// class that contains a subscriber to listen the topic "/265/odom" convert the odom message to a tf message and publish it with a broadcaster to the tftree
class TFPublisher
{
public:
    TFPublisher() {
        sub = n.subscribe("/t265/odom", 1000, &TFPublisher::odomCallback, this);

//        sub1 = n.subscribe("/tf_static", 1000, &TFPublisher::staticTFCallback, this);
//        sub1 = n.subscribe("/tf", 1000, &TFPublisher::staticTFCallback, this);
    }

    // This is the callback function that will be called when a new message is received, it convert the odom message to a tf message and publish it with a broadcaster to the tftree
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

//        printf("Received message\n");
//        printf("x: %f\n", msg->pose.pose.position.x);
//        printf("y: %f\n", msg->pose.pose.position.y);
//        printf("z: %f\n", msg->pose.pose.position.z);
//        printf("qx: %f\n", msg->pose.pose.orientation.x);
//        printf("qy: %f\n", msg->pose.pose.orientation.y);
//        printf("qz: %f\n", msg->pose.pose.orientation.z);
//        printf("qw: %f\n", msg->pose.pose.orientation.w);

        // Create a transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        transform.setRotation(q);

//        printf("Transform created\n");
//        printf("x: %f\n", transformStamped.transform.translation.x);
//        printf("y: %f\n", transformStamped.transform.translation.y);
//        printf("z: %f\n", transformStamped.transform.translation.z);
//        printf("qx: %f\n", transformStamped.transform.rotation.x);
//        printf("qy: %f\n", transformStamped.transform.rotation.y);
//        printf("qz: %f\n", transformStamped.transform.rotation.z);
//        printf("qw: %f\n", transformStamped.transform.rotation.w);
//
//        printf("Publishing transform\n");

        // Publish the transform
//        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "t265"));
        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "t265"));
    }

//    void staticTFCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
//        printf("Received static tf message\n");
//        printf("frame_id: %s\n", msg->transforms[0].header.frame_id.c_str());
//        printf("child_frame_id: %s\n", msg->transforms[0].child_frame_id.c_str());
//        printf("x: %f\n", msg->transforms[0].transform.translation.x);
//        printf("y: %f\n", msg->transforms[0].transform.translation.y);
//        printf("z: %f\n", msg->transforms[0].transform.translation.z);
//        printf("qx: %f\n", msg->transforms[0].transform.rotation.x);
//        printf("qy: %f\n", msg->transforms[0].transform.rotation.y);
//        printf("qz: %f\n", msg->transforms[0].transform.rotation.z);
//        printf("qw: %f\n", msg->transforms[0].transform.rotation.w);
//    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub1;
    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

//    ros::Subscriber sub1;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    TFPublisher tf_publisher;
    ros::spin();
    return 0;
}