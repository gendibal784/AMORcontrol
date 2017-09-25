#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#define	PI 3.141516

std_msgs::String gestos, gestos1, comando;
std_msgs::Bool stop;
std_msgs::Char change;



using namespace std;
//esta funcion se encarga de la subscripcion a los gestos
void gesture(const std_msgs::String::ConstPtr& msg_gesto) {
	gestos.data=msg_gesto->data.c_str();

}

int main(int argc, char **argv){
	ros::init(argc, argv, "modo");
	ros::NodeHandle n1;
	ros::Subscriber sub = n1.subscribe("gestos", 10, &gesture);
	ros::NodeHandle np1;
	ros::Publisher pub=np1.advertise<std_msgs::Bool>("paro",10);
	ros::NodeHandle np2;
	ros::Publisher pub2=np2.advertise<std_msgs::Char>("cambio",10);

	stop.data=false;
	change.data=3;

	while (ros::ok()){
		ros::Rate(10).sleep();
		ros::spinOnce();
		//si se detecta el gesto de cambio de modo el programa requiere confirmación
		if ((gestos.data=="cambio")&&(gestos.data!=gestos1.data)){
			stop.data=true;
			pub.publish(stop);
			cout<<"______________________________________________\n";
			cout<<"¿quieres cambiar de modo? (s=si)"<<endl;
			cin>>comando.data;
			if(comando.data=="s"){
				cout<<"______________________________________________\n";
				cout<<"¿a qué modo quieres cambiar?"<<endl;
				cout<<"    a=control posicion"<<endl;
				cout<<"    b=joystick articular"<<endl;
				cout<<"    c=joystick cartesiano"<<endl;
				cout<<"    d=ir a posicion inicial"<<endl;
				cin>>comando.data;
				if (comando.data=="a"){
					change.data=0;
				}else{	if (comando.data=="b"){
						change.data=1;
					}else{	if (comando.data=="c"){
							change.data=2;
						}else{	if (comando.data=="d"){
								change.data=3;
							}
						}
					}
				}
			}
		cout<<"______________________________________________\n";
		}
		gestos1.data=gestos.data;
		stop.data=false;
		pub.publish(stop);
		pub2.publish(change);
	}
	
}


