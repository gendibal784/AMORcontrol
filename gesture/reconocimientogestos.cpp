#include <ros/ros.h>
#include <leap_motion/leapros.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#define	PI 3.141516


leap_motion::leapros data,data1;
double dedos[5],angle[3],palma[3],dedobase;
std_msgs::String gestos,comando,mano2;
std_msgs::Bool mano;
char inicio=0,start=0;


using namespace std;

inline double getdistance(double x1, double y1, double z1, double x2, double y2, double z2);
void palmposMR(const leap_motion::leapros& msg_palm);

int main(int argc, char **argv){
	ros::init(argc, argv, "subpalm");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("leapmotion/data", 1000, &palmposMR);
	ros::NodeHandle np1;
	ros::Publisher pub=np1.advertise<std_msgs::String>("gestos",1);
	ros::NodeHandle np2;
	ros::Publisher pub2=np2.advertise<std_msgs::Bool>("mano",1);


	while (ros::ok()){
		ros::Rate(10).sleep();
		ros::spinOnce();
		if (start==0){
			cout<<"______________________________________________\n";
			cout<<"¿quieres iniciar el reconocimiento gestual? (si=s)\n Si la respuesta va a ser afirmativa sitúa la palma de la extendida perpendicularmente al eje vertical a unos 20 cm de Leap Motion\n";
			cin>>comando.data;
			if(comando.data=="s"){
				start=1;
				inicio=1;
		
			}
		}else{	
			if ((data.palmpos.x==data1.palmpos.x)&&(data.palmpos.y==data1.palmpos.y)&&(data.palmpos.z==data1.palmpos.z)){
				mano.data=false;
				mano2.data="no hay";
			}else{
				mano.data=true;
				mano2.data="si hay";
			}
		//	cout<<"data1="<<data.palmpos.x<<", "<<data.palmpos.y<<", "<<data.palmpos.z<<endl;
		//	cout<<"data2="<<data1.palmpos.x<<", "<<data1.palmpos.y<<", "<<data1.palmpos.z<<endl;
			cout<<"dedo modelo="<<dedobase<<"\n";
			cout<<"mano="<<mano2.data<<endl;
			cout<<"______________________________________________\n";
			cout<<"direction="<<data.direction.x<<" ,"<<data.direction.y<<" ,"<<data.direction.z<<"\n";
			cout<<"normal="<<data.normal.x<<" ,"<<data.normal.y<<" ,"<<data.normal.z<<"\n";
			cout<<"palm_position="<<data.palmpos.x<<" ,"<<data.palmpos.y<<" ,"<<data.palmpos.z<<"\n";
		 	cout<<"hand_ypr="<<data.ypr.x<<" ,"<<data.ypr.y<<" ,"<<data.ypr.z<<"\n";
			cout<<"thumb_tip="<<dedos[0]<<"\n";
			cout<<"index_tip="<<dedos[1]<<"\n";
			cout<<"middle_tip="<<dedos[2]<<"\n";
			cout<<"ring_tip="<<dedos[3]<<"\n";
			cout<<"pinky_tip="<<dedos[4]<<"\n";
			cout<<"______________________________________________\n";
			cout<<"angle="<<angle[0]<<" ,"<<angle[0]<<" ,"<<angle[0]<<"\n";
			cout<<"angle="<<data.middle_proximal.x<<" ,"<<data.middle_proximal.y<<" ,"<<data.middle_proximal.z<<"\n";
			cout<<"______________________________________________\n";
			cout<<"info="<<getdistance(data.thumb_tip.x,data.thumb_tip.y,data.thumb_tip.z,data.index_tip.x,data.index_tip.y,data.index_tip.z)<<endl;


			if (dedos[0]>1.3 && dedos[1]>1.5 && dedos[2]>1.50 && dedos[3]>1.5 && dedos[4]>1.3){
				gestos.data="neutral";
				cout<<"posicion neutral\n";
				cout<<"______________________________________________\n";
			}else{
				if (dedos[0]<1.2 && dedos[1]<1.2 && dedos[2]<1.2 && dedos[3]<1.2 && dedos[4]<1.1){
					cout<<"cerrar garra\n";
					gestos.data="cerrar";
					cout<<"______________________________________________\n";
				}else{	if (getdistance(data.thumb_tip.x,data.thumb_tip.y,data.thumb_tip.z,data.index_tip.x,data.index_tip.y,data.index_tip.z)<50 && dedos[2]>1.5 && dedos[3]>1.5&& dedos[4]>1.3){
						cout<<"abrir garra\n";
						gestos.data="abrir";
						cout<<"______________________________________________\n";
					}else{	if (dedos[0]>1.3 && dedos[1]>1.5 && dedos[2]>1.5 && dedos[3]<1.2 && dedos[4]<1.1){
							cout<<"inicio\n";
							gestos.data="start";
							cout<<"______________________________________________\n";
						}else{	if (dedos[0]>1.3 && dedos[1]<1.2 && dedos[2]<1.2 && dedos[3]<1.2 && dedos[4]<1.1){
								cout<<"cambiar modo\n";
								gestos.data="cambio";
								cout<<"______________________________________________\n";
							}else{
								cout<<"desconocido\n";
								gestos.data="desconocido";
								cout<<"______________________________________________\n";
							}
						}
					}
				}	
			}

			pub.publish(gestos);
			pub2.publish(mano);

			data1=data;



		}
	
	}
	
}
void palmposMR(const leap_motion::leapros& msg_palm) {
    	data=msg_palm;
	if (inicio==1){
	dedobase=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.middle_tip.x,msg_palm.middle_tip.y,msg_palm.middle_tip.z)/2;
	inicio=0;
	}
	dedos[0]=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.thumb_tip.x,msg_palm.thumb_tip.y,msg_palm.thumb_tip.z)/dedobase;
	dedos[1]=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.index_tip.x,msg_palm.index_tip.y,msg_palm.index_tip.z)/dedobase;
	dedos[2]=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.middle_tip.x,msg_palm.middle_tip.y,msg_palm.middle_tip.z)/dedobase;
	dedos[3]=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.ring_tip.x,msg_palm.ring_tip.y,msg_palm.ring_tip.z)/dedobase;
	dedos[4]=getdistance(msg_palm.palmpos.x,msg_palm.palmpos.y,msg_palm.palmpos.z,msg_palm.pinky_tip.x,msg_palm.pinky_tip.y,msg_palm.pinky_tip.z)/dedobase;
	
}
inline double getdistance(double x1, double y1, double z1, double x2, double y2, double z2){
	return sqrt(pow(x1-x2,2)+pow(y1-y2,2)/2+pow(z1-z2,2));
}
/*	angle[0]=atan2(msg_palm.middle_proximal.z-msg_palm.palmpos.z,msg_palm.middle_proximal.y-msg_palm.palmpos.y)*360/PI;
	angle[1]=atan2(msg_palm.middle_proximal.x-msg_palm.palmpos.x,msg_palm.middle_proximal.z-msg_palm.palmpos.z)*360/PI;
	angle[2]=atan2(msg_palm.middle_proximal.y-msg_palm.palmpos.y,msg_palm.middle_proximal.x-msg_palm.palmpos.x)*360/PI;*/
