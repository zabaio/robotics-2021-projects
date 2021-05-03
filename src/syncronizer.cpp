#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


//call back vecchia non più necessaria utilizzata in prototipi iniziali
void f_l_callback(const robotics_hw1::MotorSpeed::ConstPtr& msg){
    ROS_INFO("anteriore sinistra: [%f]", msg->rpm);
}
// callback vecchia utilizzata nei prototipi iniziali
void f_r_callback(const robotics_hw1::MotorSpeed::ConstPtr& msg){
    ROS_INFO("anteriore destra: [%f]", msg->rpm);
} 



// callback per restituire a schermo le velocità dei 4 motori una volta sincronizzate
void speed_callback(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                    const robotics_hw1::MotorSpeed::ConstPtr& r_r,const robotics_hw1::MotorSpeed::ConstPtr& r_l ){
    ROS_INFO ("front right: %f RPM, front left %f RMP, rear right %f RMP, rear left %f",
                f_r->rpm, f_l-> rpm, r_r-> rpm, r_l->rpm);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "syncronizer");

    ros::NodeHandle n;


    // codice vecchio utilizzato nei primi prototipi
    /*ros::Subscriber front_left = n.subscribe("motor_speed_fl", 1000, f_l_callback);
    ros::Subscriber front_right = n.subscribe("motor_speed_fr", 1000, f_r_callback);*/


    //costruzione dei 4 message filters e dei conseguenti subscriber uno per ogni ruota del robot
    message_filters::Subscriber<robotics_hw1::MotorSpeed> front_l(n, "motor_speed_fl", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> front_r(n, "motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_r(n, "motor_speed_rr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_l(n, "motor_speed_rl", 1);

    //definizione della politcy per il message filter. In questo caso è stata definita una policy exact time. Non pensavo potesse funzionare ma si è rivelata funzionante
    typedef message_filters::sync_policies::ExactTime<robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> MySyncPolicy;


    //message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> sync(front_l,front_r, rear_r,rear_l,10);

    //sincronizzazione e richiamo del callback
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), front_l, front_r, rear_r, rear_l);
    sync.registerCallback(boost::bind(&speed_callback, _1,_2,_3,_4));

    ros::spin();

return 0;

}