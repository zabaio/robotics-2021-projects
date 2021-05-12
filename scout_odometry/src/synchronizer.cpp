#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <robotics_hw1/MotorSpeed.h>

/**
 * This class represents the entire synchronizer node.
 *
 * It reads the motor speed from /motor_speed_XX,
 * and when on 4 messages that satisfy the synchronization policy arrive it publishes on:
 *  - scout_twist       geometry_msgs/TwistStamped message containing linear and angular speed of the robot
 */
class SynchronizerNode{

    public:

    SynchronizerNode();

    private:

    double v_right, v_left;     // average speed of the right and left wheels in radians/second
    double v_x, w_z;            // linear and angular velocity
    double y_0;                 // the apparent base line
    float gear_ratio = 38.3;    // motor speed -> wheels gear ratio

    ros::NodeHandle nodeHandle;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> front_l;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> front_r;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_l;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_r;
    ros::Publisher twistPublisher;

    double rpmToAngular(double rpm);
    double rpmToAvgSpeed(double v_f, double v_r);
    void publishTwist(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                      const robotics_hw1::MotorSpeed::ConstPtr& r_l, const robotics_hw1::MotorSpeed::ConstPtr& r_r);

    typedef message_filters::sync_policies::ApproximateTime
            <robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed>
            MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;
};

/**
 * The class constructor looks for the apparent baseline in the parameter server,
 * sets up synchronization callback, subscribes to topics and advertises scout_twist.
 */
SynchronizerNode::SynchronizerNode()
    : sync(MySyncPolicy(10), front_l, front_r, rear_l, rear_r)
    , twistPublisher(nodeHandle.advertise<geometry_msgs::TwistStamped>("scout_twist", 1)){

    front_l.subscribe(nodeHandle, "/motor_speed_fl", 1);
    front_r.subscribe(nodeHandle, "/motor_speed_fr", 1);
    rear_l.subscribe(nodeHandle, "/motor_speed_rl", 1);
    rear_r.subscribe(nodeHandle, "/motor_speed_rr", 1);
    nodeHandle.getParam("apparent_baseline", y_0);
    ROS_INFO("Read apparent baseline: %f", y_0);

    sync.registerCallback(boost::bind(&SynchronizerNode::publishTwist, this, _1,_2,_3,_4));
}

double SynchronizerNode::rpmToAngular(double rpm){

    return rpm * 2 * 3.141 / 60;
}

/**
 * Takes the rpm of two AgileX Scout 2.0 motors and returns the average linear speed of the two wheels.
 * It takes into account the wheel radius and the gear ratio.
 *
 * @param rpm1      First motor rpm
 * @param rpm2      Second motor rpm
 * @return          Average speed of the two wheels
 */
double SynchronizerNode::rpmToAvgSpeed(double rpm1, double rpm2){

    return (rpmToAngular(rpm1) + rpmToAngular(rpm2)) / 2 / gear_ratio * 0.1575;
}

/**
 * Callback method for synchronized motor speed messages.
 * Publishes the linear and angular twist of the robot on topic scout_twist.
 *
 * @param f_l       Front left motor speed message
 * @param f_r       Front right motor speed message
 * @param r_l       Rear left motor speed message
 * @param r_r       Rear right motor speed message
 */
void SynchronizerNode::publishTwist(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                                       const robotics_hw1::MotorSpeed::ConstPtr& r_l, const robotics_hw1::MotorSpeed::ConstPtr& r_r ){

    v_left = rpmToAvgSpeed(f_l->rpm, r_l->rpm);
    v_right = rpmToAvgSpeed(f_r->rpm, r_r->rpm);

    v_left = - v_left;                  // takes into account the fact that in the provided bags
                                        // the left motor speeds are negative when moving forward

    v_x = (v_left + v_right) / 2;       // linear velocity in x
    w_z = (-v_left + v_right) / y_0;    // angular velocity

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = v_x;
    msg.twist.angular.z = w_z;
    twistPublisher.publish(msg);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "synchronizer");

    ROS_INFO("Starting synchronizer...");
    SynchronizerNode synchronizer;
    ROS_INFO("Synchronizer spinning...");

    ros::spin();

    return 0;
}