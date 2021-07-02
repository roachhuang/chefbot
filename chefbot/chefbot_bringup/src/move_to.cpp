
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose.theta = yaw;
    pub_pose2d.publish(current_pose);
}

int main(int argc, char **argv)
{
    const double PI = 3.14159265358979323846;

    ROS_INFO("start");

    ros::init(argc, argv, "move_pub");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("odom", 1, odomCallback);
    ros::Publisher movement_pub = n.advertise("cmd_vel", 1); //for sensors the value after , should be higher to get a more accurate result (queued)
    pub_pose2d = n.advertise("turtlebot_pose2d", 1);
    ros::Rate rate(10); //the larger the value, the "smoother" , try value of 1 to see "jerk" movement

    //move forward
    ROS_INFO("move forward");
    ros::Time start = ros::Time::now();
    while (ros::ok() && current_pose.x < 1.5)
    {
        geometry_msgs::Twist move; //velocity controls move.linear.x = 0.2; //speed value m/s move.angular.z = 0; movement_pub.publish(move); ros::spinOnce(); rate.sleep(); } //turn right ROS_INFO("turn right"); ros::Time start_turn = ros::Time::now(); while(ros::ok() && current_pose.theta > -PI/2)
        {
            geometry_msgs::Twist move;
            //velocity controls
            move.linear.x = 0; //speed value m/s
            move.angular.z = -0.3;
            movement_pub.publish(move);

            ros::spinOnce();
            rate.sleep();
        }
        //move forward again
        ROS_INFO("move forward");
        ros::Time start2 = ros::Time::now();
        while (ros::ok() && current_pose.y > -1.5)
        {
            geometry_msgs::Twist move;
            //velocity controls
            move.linear.x = 0.2; //speed value m/s
            move.angular.z = 0;
            movement_pub.publish(move);

            ros::spinOnce();
            rate.sleep();
        }

        // just stop
        while (ros::ok())
        {
            geometry_msgs::Twist move;
            move.linear.x = 0;
            move.angular.z = 0;
            movement_pub.publish(move);

            ros::spinOnce();
            rate.sleep();
        }

        return 0;
    }