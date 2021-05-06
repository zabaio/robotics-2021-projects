#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <scout_odometry/parametersConfig.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <scout_odometry/State.h>
#include <geometry_msgs/Point.h>

#include <scout_odometry/SetPose.h>
#include <std_srvs/Empty.h>

/**
 * This class represents the entire integrator node.
 *
 * It reads from /scout_twist, and for each geometry_msgs/TwistStamped.msg message received publishes on two topics:
 *  - /scout_odom       A nav_msgs/Odometry.msg message containing position and twist
 *  - /scout_state      A scout_odometry/State.msg message containing odometry and integration method
 * Moreover it publishes through a transform broadcaster:
 *  - /tf               A geometry_msgs/TransformStamped message containing the transform between robot and world frame
 *
 * It also provides services to reset the robot pose and to set it to a desired value,
 * as well as a reconfigurable parameter /integration_method that can be set to Euler or Runge_Kutta.
 */
class Integrator{

    public:

    enum class Method {euler, rk};

    explicit Integrator();
    void setMethod(Method _method);
    void integrate(const geometry_msgs::TwistStamped::ConstPtr& twistStamped);
    void eulerIntegrate(const geometry_msgs::TwistStamped::ConstPtr& twistStamped);
    void rkIntegrate(const geometry_msgs::TwistStamped::ConstPtr& twistStamped);
    void publish(ros::Time stamp, geometry_msgs::Twist twist, geometry_msgs::Point position, double theta);
    bool resetPose(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool setPose(scout_odometry::SetPose::Request& req, scout_odometry::SetPose::Request& res);

    private:

    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;
    ros::Publisher odomPublisher;
    ros::Publisher statePublisher;
    ros::ServiceServer resetServer;
    ros::ServiceServer setServer;
    tf2_ros::TransformBroadcaster br;

    Method method;
    ros::Time lastStamp;
    geometry_msgs::Twist lastTwist;
    geometry_msgs::Point lastPosition;
    double lastTheta;
};

/**
 * The class constructor looks for an initial pose in the parameter server,
 * subscribes to topics and advertises topics/services.
 */
Integrator::Integrator()
    : nodeHandle()
    , lastStamp(ros::Time::now())
    , subscriber(nodeHandle.subscribe("scout_twist", 1000, &Integrator::integrate, this))
    , odomPublisher(nodeHandle.advertise<nav_msgs::Odometry>("scout_odom", 1000))
    , statePublisher(nodeHandle.advertise<scout_odometry::State>("scout_state", 1000))
    , resetServer(nodeHandle.advertiseService("reset_pose", &Integrator::resetPose, this))
    , setServer(nodeHandle.advertiseService("set_pose", &Integrator::setPose, this))
    , lastPosition()
    , lastTheta(0)
    , method(Method::euler){

    nodeHandle.getParam("initial_pose/x", lastPosition.x);
    nodeHandle.getParam("initial_pose/y", lastPosition.y);
    nodeHandle.getParam("initial_pose/theta", lastTheta);
    ROS_INFO("Read initial pose: x=%f y=%f theta=%f", lastPosition.x, lastPosition.y, lastTheta);
}

/**
 * Setter for the integration method enum attribute.
 * @param _method       The desired method
 */
void Integrator::setMethod(Integrator::Method _method) {

    method = _method;
}

/**
 * Integrates the robot position, choosing the right integration type based on the method attribute.
 * @param twistStamped      The current twist of the robot, along with a timestamp
 */
void Integrator::integrate(const geometry_msgs::TwistStamped::ConstPtr& twistStamped) {

    switch (method) {
        case Method::euler :
            eulerIntegrate(twistStamped);
            break;
        case Method::rk :
            rkIntegrate(twistStamped);
    }

}

/**
 * Integrates the robot position using Euler integration.
 * @param twistStamped      The current twist of the robot, along with a timestamp
 */
void Integrator::eulerIntegrate(const geometry_msgs::TwistStamped::ConstPtr& twistStamped) {

    ROS_INFO("Euler integrating...");
    ros::Time stamp = twistStamped->header.stamp;
    ros::Duration deltaStamp = stamp - lastStamp;

    geometry_msgs::Twist twist = twistStamped->twist;

    geometry_msgs::Point position;
    position.x = lastPosition.x + lastTwist.linear.x * deltaStamp.toSec() * cos(lastTheta);
    position.y = lastPosition.y + lastTwist.linear.x * deltaStamp.toSec() * sin(lastTheta);

    double theta = lastTheta + lastTwist.angular.z * deltaStamp.toSec();

    publish(stamp, twist, position, theta);

    lastStamp = stamp;
    lastTwist = twist;
    lastPosition = position;
    lastTheta = theta;
}

/**
 * Integrates the robot position using Runge-Kutta integration.
 * @param twistStamped      The current twist of the robot, along with a timestamp
 */
void Integrator::rkIntegrate(const geometry_msgs::TwistStamped::ConstPtr &twistStamped) {

    ROS_INFO("Runge-Kutta integrating...");
    ros::Time stamp = twistStamped->header.stamp;
    ros::Duration deltaStamp = stamp - lastStamp;

    geometry_msgs::Twist twist = twistStamped->twist;

    geometry_msgs::Point position;
    position.x = lastPosition.x + lastTwist.linear.x * deltaStamp.toSec() * cos(lastTheta + lastTwist.angular.z * deltaStamp.toSec() / 2);
    position.y = lastPosition.y + lastTwist.linear.x * deltaStamp.toSec() * sin(lastTheta + lastTwist.angular.z * deltaStamp.toSec() / 2);

    double theta = lastTheta + lastTwist.angular.z * deltaStamp.toSec();

    publish(stamp, twist, position, theta);

    lastStamp = stamp;
    lastTwist = twist;
    lastPosition = position;
    lastTheta = theta;

}

/**
 * Publishes the current robot state, odometry, and tf on the respective topics.
 * @param stamp         The current timestamp
 * @param twist         The current twist
 * @param position      The current position
 * @param theta         The current orientation, measured as the angle around the z-axis between the robot and world frames
 */
void Integrator::publish(ros::Time stamp, geometry_msgs::Twist twist, geometry_msgs::Point position, double theta) {

    ROS_INFO("Publishing stamp=%f twist.x=%f twist.z=%f position.x=%f position.y=%f theta=%f", stamp.toSec(), twist.linear.x, twist.angular.z, position.x, position.y, theta);
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    q.normalize();
    ROS_INFO("theta=%f x=%f y=%f z=%f w=%f", theta, q.x(), q.y(), q.z(), q.w());
    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    ROS_INFO("x=%f y=%f z=%f w=%f", orientation.x, orientation.y, orientation.z, orientation.w);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.twist.twist = twist;
    odom.pose.pose.position = position;
    odom.pose.pose.orientation = orientation;
    odomPublisher.publish(odom);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = stamp;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "scout";
    transformStamped.transform.translation.x = position.x;
    transformStamped.transform.translation.y = position.y;
    transformStamped.transform.rotation = orientation;
    br.sendTransform(transformStamped);

    scout_odometry::State state;
    state.odom = odom;
    state.method = (method == Method::euler) ? "euler" : "rk";

    statePublisher.publish(state);

}

/**
 * Callback function for the ResetPose service.
 * Since no information is transferred, we use the service std_srvs/Empty.
 * @param req       The service arguments (none)
 * @param res       The service response (none)
 * @return          Always true
 */
bool Integrator::resetPose(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){

    lastPosition.x = 0;
    lastPosition.y = 0;
    lastTheta = 0;
    lastTwist.linear.x = 0;
    lastTwist.angular.z = 0;
    ROS_INFO("ResetPose request received: x=%f, y=%f, theta=%f", lastPosition.x, lastPosition.y, lastTheta);
    return true;
}

/**
 * Callback function for the SetPose service.
 * We use a custom service scout_odometry/SetPose.
 * @param req       The service arguments (x, y, theta)
 * @param res       The service response (none)
 * @return          Always true
 */
bool Integrator::setPose(scout_odometry::SetPose::Request& req, scout_odometry::SetPose::Request& res){

    lastPosition.x = req.x;
    lastPosition.y = req.y;
    lastTheta = req.theta;
    lastTwist.linear.x = 0;
    lastTwist.angular.z = 0;
    ROS_INFO("SetPose request received: x=%f, y=%f, theta=%f", lastPosition.x, lastPosition.y, lastTheta);
    return true;
}

/**
 * The callback function for the dynamic_reconfigure server.
 * It changes Integrator::method according to the selected integration_method parameter.
 * @param integrator    The integrator instance to be modified
 * @param config        The config containing the method choice
 * @param level         The dynamic_reconfigure level bitmask
 */
void setMethod(Integrator& integrator, scout_odometry::parametersConfig& config, uint32_t level) {

    if (config.integration_method == 0) {
        integrator.setMethod(Integrator::Method::euler);
        ROS_INFO("SetMethod request received. Method is: Euler");
    }
    if (config.integration_method == 1) {
        integrator.setMethod(Integrator::Method::rk);
        ROS_INFO("SetMethod request received. Method is: Runge_Kutta");
    }
}

/**
 * In main we create the node, the Integrator instance, launch the dynamic_reconfigure server, and spin.
 */
int main(int argc, char **argv){

    ros::init(argc, argv, "integrator");

    Integrator integrator;

    dynamic_reconfigure::Server<scout_odometry::parametersConfig> server;
    dynamic_reconfigure::Server<scout_odometry::parametersConfig>::CallbackType paramCallback;
    paramCallback = boost::bind(&setMethod, boost::ref(integrator), _1, _2);
    server.setCallback(paramCallback);
    ROS_INFO("Integrator spinning...");

    ros::spin();

    return 0;
}