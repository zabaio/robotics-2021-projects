#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf_reconfigure/parametersConfig.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

geometry_msgs::TransformStamped transformStamped;

void setMethod(tf_reconfigure::parametersConfig& config, uint32_t level) {

    tf2::Quaternion q;
    q.setRPY(config.ori_r, config.ori_p, config.ori_y);
    q.normalize();
    geometry_msgs::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    transformStamped.transform.rotation = orientation;
    transformStamped.transform.translation.x = config.pos_x;
    transformStamped.transform.translation.y = config.pos_y;
    transformStamped.transform.translation.z = config.pos_z;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "tf_pub");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    tf2_ros::TransformBroadcaster br;
    dynamic_reconfigure::Server<tf_reconfigure::parametersConfig> server;
    dynamic_reconfigure::Server<tf_reconfigure::parametersConfig>::CallbackType paramCallback;
    paramCallback = boost::bind(&setMethod, _1, _2);
    server.setCallback(paramCallback);

    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "odom";

    while (ros::ok()){

        transformStamped.header.stamp = ros::Time::now();

        br.sendTransform(transformStamped);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}