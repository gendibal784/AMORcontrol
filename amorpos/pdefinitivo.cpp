#include <ros/ros.h>
#include <iostream>
#include <leap_motion/leapros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#define	PI 3.141516

geometry_msgs::Twist yarpmsg,vel,offs,distancia,angulo,joystick,joy,initial,artpose,artAMOR,vel1;
leap_motion::leapros data;
char inicio=0,Kleap=1.5,change,change1=3,guardaroffs=0;
std_msgs::String gestos,gestos1,comando;
std_msgs::Bool mode,pos,mano,stop;
bool question;

using namespace std;

void AMORpos(const geometry_msgs::Twist& msg);
void palmposMR(const leap_motion::leapros& msg_palm);
void gesture(const std_msgs::String::ConstPtr& msg_gesto);
void ARTposition(const geometry_msgs::Twist& msg);
void hand(const std_msgs::Bool& msg);
void parada(const std_msgs::Bool& msg);
void cambiarmodo(const std_msgs::Char& msg);
inline double getlineardistance(double x1, double x2);
inline double getvelocity(double x1);
inline double RAD2DEG(double x1);

int main(int argc, char **argv){
	ros::init(argc, argv, "subpos");
	ros::NodeHandle n1;
	ros::Subscriber sub = n1.subscribe("posicion2", 10, &AMORpos);
	ros::NodeHandle n2;
	ros::Subscriber sub2 = n2.subscribe("leapmotion/data", 10, &palmposMR);
	ros::NodeHandle n3;
	ros::Subscriber sub3 = n3.subscribe("gestos", 10, &gesture);
	ros::NodeHandle n8;
	ros::Subscriber sub4 = n8.subscribe("artpos", 10, &ARTposition);
	ros::NodeHandle n9;
	ros::Subscriber sub5 = n9.subscribe("mano", 10, &hand);
	ros::NodeHandle n10;
	ros::Subscriber sub6 = n10.subscribe("paro", 10, &parada);
	ros::NodeHandle n11;
	ros::Subscriber sub7 = n11.subscribe("cambio", 10, &cambiarmodo);
	ros::NodeHandle n4;
	ros::Publisher pub=n4.advertise<geometry_msgs::Twist>("chatter",10);
	ros::NodeHandle n5;
	ros::Publisher pub2=n5.advertise<std_msgs::String>("garra",10);
	ros::NodeHandle n6;
	ros::Publisher pub3=n6.advertise<std_msgs::Bool>("Mode",10);
	ros::NodeHandle n7;
	ros::Publisher pub4=n7.advertise<std_msgs::Bool>("return",10);


	//posición inicial
/*	initial.linear.x=-4;
	initial.linear.y=-592;
	initial.linear.z=164;
	initial.angular.x=-172;
	initial.angular.y=-5;
	initial.angular.z=13;*/
	initial.linear.x=RAD2DEG(-0.458);
	initial.linear.y=RAD2DEG(-0.053);
	initial.linear.z=RAD2DEG(-2.255);
	initial.angular.x=RAD2DEG(3.125);
	initial.angular.y=RAD2DEG(1.713);
	initial.angular.z=RAD2DEG(-1.589);



//aqui el programa no inicia su funcionamiento hasta que se realiza el gesto correcto y el robot se encuentra en la posicion inicial
	while((ros::ok())&&(gestos.data!="start")){
		ros::Rate(8).sleep();
		ros::spinOnce();	
	};


//una vez iniciado el programa se inicializan todas las variables de funcionamiento del programa
	inicio=1;
	change=3;
	mode.data=false;
	guardaroffs=1;
	//offs joystick
	joystick.linear.x=0;
	joystick.linear.y=200;
	joystick.linear.z=0;

	while (ros::ok()){

		//se establece la frecuencia y se pasa una vez por las subscripciones
		ros::Rate(8).sleep();
		ros::spinOnce();

			if((change!=change1)&&(change==0)){
				guardaroffs=1;
			}

			//comprueba el modo y utiliza el correcto
			switch (change){

				case 0:
					cout<<"______________________________________________\n";
					cout<<"Modo control de posicion\n";	
					mode.data=false;
					pos.data=false;
					cout<<"______________________________________________\n";
					cout<<"robot="<<yarpmsg.linear.x<<" ,"<<yarpmsg.linear.y<<" ,"<<yarpmsg.linear.z<<"\n";
				//	cout<<"offs="<<offs.linear.x<<" ,"<<offs.linear.y<<" ,"<<offs.linear.z<<"\n";
				//	cout<<"______________________________________________\n";
					cout<<"palm_position="<<data.palmpos.x<<" ,"<<data.palmpos.z<<" ,"<<data.palmpos.y<<"\n";
				//	cout<<"______________________________________________\n";
					cout<<"gesto="<<gestos.data<<"\n";
					if (getlineardistance(data.palmpos.x,yarpmsg.linear.x)<20){
						vel.linear.x=0;
						distancia.linear.x=0;
					}else{
						distancia.linear.x=(data.palmpos.x-yarpmsg.linear.x)/50;
						vel.linear.x=getvelocity(distancia.linear.x)/6;
					}
			
					if (getlineardistance(data.palmpos.z,yarpmsg.linear.y)<20){
						vel.linear.y=0;
						distancia.linear.y=0;
					}else{
						distancia.linear.y=(data.palmpos.z-yarpmsg.linear.y)/50;
						vel.linear.y=getvelocity(distancia.linear.y)/6;
					}
		
					if (getlineardistance(data.palmpos.y,yarpmsg.linear.z)<20){
						vel.linear.z=0;
						distancia.linear.z=0;
					}else{
						distancia.linear.z=(data.palmpos.y-yarpmsg.linear.z)/40;
						vel.linear.z=getvelocity(distancia.linear.z)/6;
					}
				//	cout<<"______________________________________________\n";
					cout<<"linear="<<vel.linear.x<<" ,"<<vel.linear.y<<" ,"<<vel.linear.z<<"\n";
				//	cout<<"______________________________________________\n";
				//	cout<<"prueba="<<prueba.linear.x<<" ,"<<prueba.linear.y<<" ,"<<prueba.linear.z<<"\n";
					cout<<"______________________________________________\n";
					if (mano.data==false){
					vel.linear.x=0;vel.linear.y=0;vel.linear.z=0;
					gestos.data="desconocido";
					}
					vel1=vel;
					vel.linear.x=-(vel1.linear.x*cos(yarpmsg.angular.x)-vel1.linear.y*sin(yarpmsg.angular.x));
					vel.linear.y=-(vel1.linear.x*sin(yarpmsg.angular.x)+vel1.linear.y*cos(yarpmsg.angular.x));
					//					vel.linear.x=0;vel.linear.y=0;vel.linear.z=0;
					break;
				case 1:
					mode.data=true;
					pos.data=false;
					cout<<"______________________________________________\n";
					cout<<"Modo joystick articular\n";	
					cout<<"______________________________________________\n";
				//	cout<<"palm_position="<<data.palmpos.z<<" ,"<<data.palmpos.y<<" ,"<<data.palmpos.x<<"\n";
					cout<<"robot="<<RAD2DEG(yarpmsg.angular.x)<<" ,"<<RAD2DEG(yarpmsg.angular.y)<<" ,"<<RAD2DEG(yarpmsg.angular.z)<<"\n";
					cout<<"______________________________________________\n";
					cout<<"gesto="<<gestos.data<<"\n";


		
					if (getlineardistance(data.palmpos.x,joystick.linear.z)<25){
						vel.linear.x=0;
						joy.linear.x=0;
					}else{
						joy.linear.x=-(data.palmpos.x-joystick.linear.z)/50;
						vel.linear.x=getvelocity(joy.linear.x)/20;
					}

					if (getlineardistance(data.palmpos.y,joystick.linear.y)<25){
						vel.linear.z=0;
						joy.linear.z=0;
					}else{
						joy.linear.z=-(data.palmpos.y-joystick.linear.y)/50;
						vel.linear.z=getvelocity(joy.linear.z)/30;
					}
		
				//	cout<<"______________________________________________\n";
					cout<<"linear="<<vel.linear.x<<" ,"<<vel.linear.z<<"\n";
				//	cout<<"prueba="<<joyprueba.linear.x<<"\n";
					cout<<"______________________________________________\n";
					if (mano.data==false){
					vel.linear.x=0;vel.linear.y=0;vel.linear.z=0;
					gestos.data="desconocido";
					}
					break;
				case 2:
					mode.data=false;
					pos.data=false;
					cout<<"______________________________________________\n";
					cout<<"Modo joystick cartesiano\n";	
					cout<<"______________________________________________\n";
					cout<<"palm_position="<<data.palmpos.z<<" ,"<<data.palmpos.y<<" ,"<<data.palmpos.x<<"\n";
				//	cout<<"______________________________________________\n";
					cout<<"gesto="<<gestos.data<<"\n";


		
					if (getlineardistance(data.palmpos.x,joystick.linear.x)<25){
						vel.linear.x=0;
						joy.linear.x=0;
					}else{
						joy.linear.x=(data.palmpos.x-joystick.linear.x)/50;
						vel.linear.x=getvelocity(joy.linear.x)/3.5;
					}
					if (getlineardistance(data.palmpos.z,joystick.linear.z)<25){
						vel.linear.y=0;
						joy.linear.y=0;
					}else{
						joy.linear.y=(data.palmpos.z-joystick.linear.z)/40;
						vel.linear.y=getvelocity(joy.linear.y)/3.5;
					}
					if (getlineardistance(data.palmpos.y,joystick.linear.y)<25){
						vel.linear.z=0;
						joy.linear.z=0;
					}else{
						joy.linear.z=(data.palmpos.y-joystick.linear.y)/50;
						vel.linear.z=getvelocity(joy.linear.z)/3.5;
					}
					cout<<"linear="<<vel.linear.x<<", "<<vel.linear.y<<", "<<vel.linear.z<<"\n";
					vel1=vel;
					vel.linear.x=-(vel1.linear.x*cos(yarpmsg.angular.x)-vel1.linear.y*sin(yarpmsg.angular.x));
					vel.linear.y=-(vel1.linear.x*sin(yarpmsg.angular.x)+vel1.linear.y*cos(yarpmsg.angular.x));
		
				//	cout<<"______________________________________________\n";
					cout<<"linear="<<vel.linear.x<<", "<<vel.linear.y<<", "<<vel.linear.z<<"\n";
				//	cout<<"prueba="<<joyprueba.linear.x<<"\n";
					cout<<"______________________________________________\n";
					if (mano.data==false){
						vel.linear.x=0;vel.linear.y=0;vel.linear.z=0;
						gestos.data="desconocido";
					}
					break;
				case 3:
					mode.data=true;
					pos.data=true;
					cout<<"______________________________________________\n";
					cout<<"Posición inicial\n";	

					if (getlineardistance(initial.linear.x,artAMOR.linear.x)<4){
						vel.linear.x=0;
						angulo.linear.x=0;
					}else{
						angulo.linear.x=(initial.linear.x-artAMOR.linear.x)/50;
						vel.linear.x=getvelocity(angulo.linear.x)/50;
					}
					if (getlineardistance(initial.linear.y,artAMOR.linear.y)<4){
						vel.linear.y=0;
						angulo.linear.y=0;
					}else{
						angulo.linear.y=(initial.linear.y-artAMOR.linear.y)/50;
						vel.linear.y=getvelocity(angulo.linear.y)/50;
					}
					if (getlineardistance(initial.linear.z,artAMOR.linear.z)<4){
						vel.linear.z=0;
						angulo.linear.z=0;
					}else{
						angulo.linear.z=(initial.linear.z-artAMOR.linear.z)/50;
						vel.linear.z=getvelocity(angulo.linear.z)/50;
					}
					if (getlineardistance(initial.angular.x,artAMOR.angular.x)<4){
						vel.angular.x=0;
						angulo.angular.x=0;
					}else{
						angulo.angular.x=(initial.angular.x-artAMOR.angular.x)/50;
						vel.angular.x=getvelocity(angulo.angular.x)/50;
					}
					if (getlineardistance(initial.angular.y,artAMOR.angular.y)<4){
						vel.angular.y=0;
						angulo.angular.y=0;
					}else{
						angulo.angular.y=(initial.angular.y-artAMOR.angular.y)/50;
						vel.angular.y=getvelocity(angulo.angular.y)/50;
					}
					if (getlineardistance(initial.angular.z,artAMOR.angular.z)<4){
						vel.angular.z=0;
						angulo.angular.z=0;
					}else{
						angulo.angular.z=(initial.angular.z-artAMOR.angular.z)/50;
						vel.angular.z=getvelocity(angulo.angular.z)/50;
					}
					cout<<"______________________________________________\n";
				//	cout<<"art AMOR="<<artAMOR.linear.x<<" ,"<<artAMOR.linear.y<<" ,"<<artAMOR.linear.z<<" ,"<<artAMOR.angular.x<<" ,"<<artAMOR.angular.y<<" ,"<<artAMOR.angular.z<<"\n";
				//	cout<<"initial="<<initial.linear.x<<" ,"<<initial.linear.y<<" ,"<<initial.linear.z<<" ,"<<initial.angular.x<<" ,"<<initial.angular.y<<" ,"<<initial.angular.z<<"\n";
				//	cout<<"______________________________________________\n";
					cout<<"vel="<<vel.linear.x<<" ,"<<vel.linear.y<<" ,"<<vel.linear.z<<" ,"<<vel.angular.x<<" ,"<<vel.angular.y<<" ,"<<vel.angular.z<<"\n";
				//	artpose.linear.x=0;artpose.linear.y=0;artpose.linear.z=0;artpose.angular.x=0;artpose.angular.y=0;artpose.angular.z=0;
					if (mano.data==false){
						gestos.data="desconocido";
					}
					break;

				default:
					cout<<"Error";
					break;

			}	
			


			change1=change;
			if (stop.data==true){
				vel.linear.x=0;vel.linear.y=0;vel.linear.z=0;vel.angular.x=0;vel.angular.y=0;vel.angular.z=0;
				gestos.data="desconocido";
			}

			pub3.publish(mode);
			pub4.publish(pos);
			pub.publish(vel);
			pub2.publish(gestos);		
		

	}
}
//esta función se encarga de la subscripción a la posicion de AMOR
void AMORpos(const geometry_msgs::Twist& msg) {
	yarpmsg=msg;
	if (inicio==1){
		if (guardaroffs){
			//offs garra
			offs.linear.y=yarpmsg.linear.y;
			offs.linear.x=yarpmsg.linear.x;
			offs.linear.z=yarpmsg.linear.z-200;
			guardaroffs=0;
		}
		yarpmsg.linear.x=-(yarpmsg.linear.x-offs.linear.x);
		yarpmsg.linear.y=-(yarpmsg.linear.y-offs.linear.y);
		yarpmsg.linear.z=(yarpmsg.linear.z-offs.linear.z);
	}

	
}
//esta función se encarga de la subscripción a la posicion de AMOR
void ARTposition(const geometry_msgs::Twist& msg) {
	artAMOR=msg;
}

//esta funcion se encarga de la subscripción a Leap Motion
void palmposMR(const leap_motion::leapros& msg_palm) {
    	data=msg_palm;
	if((data.palmpos.x>70)||(data.palmpos.x<-70)){
		if(data.palmpos.x>0){
			data.palmpos.x=70;
		}else{
			data.palmpos.x=-70;
		}
	}
	if((data.palmpos.y>300)||(data.palmpos.y<145)){
		if(data.palmpos.y>200){
			data.palmpos.y=300;
		}else{
			data.palmpos.y=145;
		}
	}
	if((data.palmpos.z>70)||(data.palmpos.z<-70)){
		if(data.palmpos.z>0){
			data.palmpos.z=70;
		}else{
			data.palmpos.z=-70;
		}
	}
	data.palmpos.x=data.palmpos.x;
	data.palmpos.y=data.palmpos.y;
	data.palmpos.z=-data.palmpos.z;	
}
//esta funcion se encarga de la subscripcion a los gestos
void gesture(const std_msgs::String::ConstPtr& msg_gesto) {
	gestos.data=msg_gesto->data.c_str();

}
void hand(const std_msgs::Bool& msg){
	mano=msg;
}
void parada(const std_msgs::Bool& msg){
	stop=msg;
}
void cambiarmodo(const std_msgs::Char& msg){
	change=msg.data;
}
//esta funcion obtiene la distantcia entre dos puntos
inline double getlineardistance(double x1, double x2){
	return sqrt(pow(x1-x2,2));
}
//esta funcion obtiene la velocidad
inline double getvelocity(double x1){
	if (x1<0){	
		return -pow(pow(x1,2),0.1);
	}else{
		return pow(pow(x1,2),0.1);
	}
}
//esta funcion obtiene la velocidad
inline double RAD2DEG(double x1){

		return x1*180/PI;

}
