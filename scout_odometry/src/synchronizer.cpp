#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h"


using namespace message_filters;



class speed_node{

    public:
        geometry_msgs::TwistStamped msg;
        

    private:

        double v_right, v_left; // speed of the right and left wheels in radiants/second
        double v_x, w_z, v_y = 0; //velocity in y and in x
        double y0 = 0.583; // half of the apparent base line
        float gear_ratio =37.5;
        
        
        ros::NodeHandle n; 

        //definition made to use the message filters
        message_filters::Subscriber<robotics_hw1::MotorSpeed> front_l;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> front_r;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_r;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_l;
        //definition of the filter policy
        typedef message_filters::sync_policies::ExactTime<robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        //definition in order to use a dynamic pointer
        boost::shared_ptr<Sync> sync;
        ros::Publisher pub_velocita;
    
    public:
        //standard contructor
        speed_node(){
            front_l.subscribe(n, "/motor_speed_fl", 1);
            front_r.subscribe(n, "/motor_speed_fr", 1);
            rear_r.subscribe(n, "/motor_speed_rr", 1);
            rear_l.subscribe(n, "/motor_speed_rl", 1);
            pub_velocita = n.advertise<geometry_msgs::TwistStamped>("scout_twist", 1);
            n.getParam("/apparent_baseline", y0);

            sync.reset(new Sync(MySyncPolicy(10), front_l, front_r, rear_r, rear_l));
            sync->registerCallback(boost::bind(&speed_node::speed_callback, this, _1,_2,_3,_4));
    }

        //metods definitions

        //subscriber callback
        void speed_callback(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                            const robotics_hw1::MotorSpeed::ConstPtr& r_r,const robotics_hw1::MotorSpeed::ConstPtr& r_l );
        
        private:
        double calcolo_radiantis(double girim); // method that campute the speed in radiant/second
        double calcolo_velocita_ruota(double v_f, double v_r); //method that compute the linear speed
};

//method to tranform rmp in radiants/second
double speed_node::calcolo_radiantis(double girim){

    return girim*2*3.141/60;  //rpm two times pi / 60
}

//method that returns the speed of a single wheel
double speed_node::calcolo_velocita_ruota(double v_f, double v_r){

    double  v_ruota;

    v_ruota = (((calcolo_radiantis(v_f) + calcolo_radiantis(v_r))/2)/gear_ratio)*0.1575;// mean of front and rear velocity divided the gear ratio times the wheel's radius

    return v_ruota;
}

//subscriber's callback
void speed_node::speed_callback(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                            const robotics_hw1::MotorSpeed::ConstPtr& r_r,const robotics_hw1::MotorSpeed::ConstPtr& r_l ){
    v_left = speed_node::calcolo_velocita_ruota(f_l->rpm,  r_l->rpm); //left and right speed
    v_right = speed_node::calcolo_velocita_ruota(f_r->rpm,  r_r->rpm);

    v_left = -v_left;

    v_x = (v_left + v_right)/2;       //linear velocity in x
    w_z = (-v_left+v_right)/(2*y0);   //angular velocity

    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = v_x;         //velocity message
    msg.twist.angular.z = w_z;
    msg.twist.linear.y = v_y;
       
    pub_velocita.publish(msg);        //publishing

}


int main(int argc, char **argv){

    ros::init(argc, argv, "synchronizer");

    speed_node sub_pub;     //object creation 

    ROS_INFO("Synchronizer spinning...");
    ros::spin();

    return 0;

}