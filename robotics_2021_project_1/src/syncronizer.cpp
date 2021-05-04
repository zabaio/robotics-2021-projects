#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "geometry_msgs/TwistStamped.h"
#include "robotics_hw1/MotorSpeed.h"


using namespace message_filters;



class Nodo_velocita{

    public:
        geometry_msgs::TwistStamped msg;
        

    private:

        double v_right, v_left; // velocità della ruota destra e sinistra espresse in radianti su secondo
        double v_x, w_z; //velocity in y and in x
        double y0 = 0.583; // half of the apparent base line
        
        
        ros::NodeHandle n; 

        //definizioni per l'utilizzo dei message filter
        message_filters::Subscriber<robotics_hw1::MotorSpeed> front_l;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> front_r;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_r;
        message_filters::Subscriber<robotics_hw1::MotorSpeed> rear_l;
        //definizione della politica del filtro
        typedef message_filters::sync_policies::ExactTime<robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed,robotics_hw1::MotorSpeed> MySyncPolicy;
        typedef Synchronizer<MySyncPolicy> Sync;
        //definizione per l'utilizzo di un puntatore dinamico
        boost::shared_ptr<Sync> sync;
        ros::Publisher pub_velocita;
    
    public:
        //costruttore standard
        Nodo_velocita(){
            front_l.subscribe(n, "motor_speed_fl", 1);
            front_r.subscribe(n, "motor_speed_fr", 1);
            rear_r.subscribe(n, "motor_speed_rr", 1);
            rear_l.subscribe(n, "motor_speed_rl", 1);
            pub_velocita =n.advertise<geometry_msgs::TwistStamped>("pub_velocita", 1);


            sync.reset(new Sync(MySyncPolicy(10), front_l, front_r, rear_r, rear_l));
            sync->registerCallback(boost::bind(&Nodo_velocita::speed_callback, this, _1,_2,_3,_4));

           
    }

        //definizione dei metodi

        //callback per il subscriber
        void speed_callback(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                            const robotics_hw1::MotorSpeed::ConstPtr& r_r,const robotics_hw1::MotorSpeed::ConstPtr& r_l );
        
        private:
        double calcolo_radiantis(double girim); // funzione per il calcolo dei radianti al secondo
        double calcolo_velocita_ruota(double v_f, double v_r); //funzione epr il calcolo della velocità lineare di una singola ruota intesa compe prodotto tra la velocità angolare e il raggio
};

//nodo per passare da rpm a radianti per secondo
double Nodo_velocita::calcolo_radiantis(double girim){

    return girim*2*3.141/60;  //rpm per 2 volte pigreco fratto 60
}

//nodo per il calcolo della velocità della singola ruota
double Nodo_velocita::calcolo_velocita_ruota(double v_f, double v_r){

    double  v_ruota;

    v_ruota = (((calcolo_radiantis(v_f) + calcolo_radiantis(v_r))/2)/37.5)*0.1575;// media della velocità anteriore e posteriore fratto il rapporto di riduzione per il raggio

    return v_ruota;
}

//callback per il subscriber
void Nodo_velocita::speed_callback(const robotics_hw1::MotorSpeed::ConstPtr& f_l, const robotics_hw1::MotorSpeed::ConstPtr& f_r,
                            const robotics_hw1::MotorSpeed::ConstPtr& r_r,const robotics_hw1::MotorSpeed::ConstPtr& r_l ){
    v_right = Nodo_velocita::calcolo_velocita_ruota(f_l->rpm,  r_l->rpm); //calcolo della velocità destra e sinistra
    v_left = Nodo_velocita::calcolo_velocita_ruota(f_r->rpm,  r_r->rpm);

    v_x = (v_left + v_right)/2;       //calcolo della velocità lineare in x
    w_z = (-v_left+v_right)/(2*y0);   //calcolo della velocità angolare rispetto a z

    msg.twist.linear.x = v_x;         //scrittura delle velocità nel messaggio
    msg.twist.angular.z = w_z;
 ROS_INFO ("Velocità x %f, \n angolare in z %f \n\n", v_right, v_left);
       
    pub_velocita.publish(msg);        //publicazione del messaggio

}


int main(int argc, char **argv){

    ros::init(argc, argv, "syncronizer");

    Nodo_velocita sub_pub;     //creazione dell'oggetto 

    ros::spin();

return 0;

}