/**     2017-12-17
 *      michealchen         michealchen17@163.com
 * this is a robot simulator. this node keep publishing tf and pose.
 * when a cmd_vel arrival, it will change the move status of the robot
 *
 * just for fun
 * */
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

void Q2Euler(tf::Quaternion &q, double& roll, double& pitch, double& yaw)
{
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy);
}

class robot
{
public:
    ros::Publisher pose_pub;
private:
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    tf::TransformBroadcaster br;
    unsigned int msg_seq_pose;
    unsigned int msg_seq_twist;
    double time1;
    double time2;

public:
    robot();
    ~robot();
    void twistCallback(const geometry_msgs::TwistStamped &msg);
    void poseCallback(const ros::TimerEvent& tE);

};

robot::robot():
    msg_seq_pose(0),
    msg_seq_twist(0),
    time1(ros::Time::now().toSec()),
    time2(time1)
{
    pose.header.frame_id = "odom";
    pose.header.stamp = ros::Time::now();
    pose.header.seq = msg_seq_pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.header.seq = msg_seq_twist;
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    twist.twist.angular.x = 0;
    twist.twist.angular.y = 0;
    twist.twist.angular.z = 0;

}
robot::~robot()
{

}

void robot::twistCallback(const geometry_msgs::TwistStamped &msg)
{
    twist = msg;
}

void robot::poseCallback(const ros::TimerEvent &tE)
{

    tf::Quaternion poseQ;
    poseQ.setW(pose.pose.orientation.w);
    poseQ.setX(pose.pose.orientation.x);
    poseQ.setY(pose.pose.orientation.y);
    poseQ.setZ(pose.pose.orientation.z);
    double roll, pitch, yaw;
    Q2Euler(poseQ, roll, pitch, yaw);
    pose.header.stamp = ros::Time::now();
    time2 = pose.header.stamp.toSec();
    pose.header.seq = msg_seq_pose;
    double diffTime = time2 - time1;
    double yaw2 = yaw + diffTime*twist.twist.angular.z;
    double mYaw = (yaw+yaw2)/2;
    pose.pose.position.x += diffTime*twist.twist.linear.x*cos(mYaw);
    pose.pose.position.y += diffTime*twist.twist.linear.x*sin(mYaw);
    poseQ.setEuler(0,0,yaw2);
    pose.pose.orientation.w = poseQ.w();
    pose.pose.orientation.x = poseQ.x();
    pose.pose.orientation.y = poseQ.y();
    pose.pose.orientation.z = poseQ.z();
    tf::StampedTransform tran;
    tran.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0));
    tran.setRotation(poseQ);
    br.sendTransform(tf::StampedTransform(tran, ros::Time::now(),"odom", "base_link"));
    pose_pub.publish(pose);
    time1 = time2;
    msg_seq_pose++;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_publish");
    ros::NodeHandle nh;
    robot rb;
    rb.pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_poseStamped",1);
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, &robot::twistCallback, &rb);
    ros::Timer timer_pose = nh.createTimer(ros::Duration(0.1),  &robot::poseCallback, &rb);

    ros::spin();

    return 0;
}


