// cartesian_rate_AMOR.cpp : v0.3 : Defines the entry point for the application.
//

#include <iostream>
#include <iomanip>
#include <assert.h>
//#include <conio.h>
//#include <windows.h>

#include <stdexcept>
#include <stdlib.h>
#include <termios.h>
#include <signal.h>
struct termios ots;
#include <fcntl.h>
#include <unistd.h>

#include <fstream>
#include <time.h>
#include <vector>

// AMOR API
#include "amor.h"

// YARP
#include "yarp/os/all.h"

//Namespace for ROS

using namespace std;

//ROS variables
#include "Twist.h"
#include "String.h"
#include "Bool.h"
Twist rdata,sdata,artdata;
String garradata;
Bool mode,pose;


// GLOBALS
const double g_translationGain = 55.0;
const double g_yawGain = 0.0; //0.015; // OFF
const double g_pitchGain = 0.0; //0.0025; // OFF
const double g_rollGain = 0.0; //0.3; // OFF
yarp::os::Network g_yarp;
bool _abort = false;
AMOR_VECTOR7 g_jointVelocity;
AMOR_VECTOR7 g_jointPosition;
AMOR_VECTOR7 g_cartVelocity;
AMOR_VECTOR7 g_cartPositions;
AMOR_JOINT_INFO g_jointInfo[AMOR_NUM_JOINTS];
AMOR_HANDLE g_hRobot	= AMOR_INVALID_HANDLE;

// HELPER MACROS

#define DEG2RAD(a)		(PI*a/180.0)
#define RAD2DEG(a)		(180.0*a/PI)
#define MM2M(a)		    (a/1000.0)

// CONSTANTS

static const real PI  = 3.141592653589793238462643383279502884197169399375105820974944;
static const real PI2 = 6.283185307179586476925286766559005768394338798750211641949888;
static const real VELOCITY_STEP_JOINT = 0.01;
static const real VELOCITY_STEP_CART_LIN = 10.0;
static const real VELOCITY_STEP_CART_ANG = 0.1;

const char* JOINT_NAMES[7] = { "A1", "A2", "A25", "A3", "A4", "A5", "A6"};
enum { A1 = 0, A2, A25, A3, A4, A5, A6 };

const real INITIAL_POSITION[AMOR_NUM_JOINTS] = {
DEG2RAD(180), DEG2RAD(-10), DEG2RAD(0), DEG2RAD(204), DEG2RAD(180), DEG2RAD(100), DEG2RAD(-90) }; // TODO: CHECK RANGE FOR JOINT 3

// FUNCTION DECLARATIONS

void ProcessKeyPress(char key);
bool KeyboardReadKey(char *key);
void Warning(unsigned joint, unsigned warning);
void print_help(void);
void print_separator(void);
void mssleep(int ms);
void ttyset(void);
void ttyreset(int);
double enforceRange(double in);

// FUNCTION DEFINITIONS

/* returns TRUE if a key has been pressed, FALSE otherwise. If a key has been pressed, the
* corresponding ASCII code for that key is copied to the provided key address */
bool KeyboardReadKey(char *key)
{
#ifdef WIN32
	if(_kbhit() == 0) 
		return false;
	*key = _getch();
	return true;
#endif // WIN32
#ifdef UNIX
	if(read(STDIN_FILENO, key, 1) < 1)
		return false;
	return true;
#endif // UNIX
}

void Warning(unsigned joint, unsigned warning)
{
	std::cout << "WARNING at " << JOINT_NAMES[joint] << ": ";
	switch(warning)
	{
	case AMOR_NO_WARNING:               std::cout << "No warning" << std::endl;                         break;
	case AMOR_WARN_TEMPERATURE:         std::cout << "Warning: temperature too high" << std::endl;      break;
	case AMOR_WARN_LOWER_END_SWITCH:    std::cout << "Warning: lower endswitch" << std::endl;           break;
	case AMOR_WARN_UPPER_END_SWITCH:    std::cout << "Warning: upper endswitch" << std::endl;           break;
	case AMOR_WARN_BUFFER_EMPTY:        std::cout << "Warning: setpoint buffer empty" << std::endl;     break;
	default:                            std::cout << "Warning: unknown warning occurred" << std::endl;  break;
	};
}

void HandleError()
{
	std::cout << "ERROR " << amor_errno() << ": " << amor_error() << std::endl;
	_abort = true;
}

bool checkBottleForDouble(yarp::os::Bottle* in_bottle, std::string in_tag, double &out_double)
{ 

	// if tag is not found or value is not double, assume that is zero, for safety
	if(!(in_bottle->find(in_tag.c_str()).isNull())) {
		if((in_bottle->find(in_tag.c_str())).isDouble()) {
			out_double = (in_bottle->find(in_tag.c_str())).asDouble();
			return true;
		}
	}
	else{
		out_double = 0.0;
		return false;
	} 

}

bool checkIfZero(std::vector<double> in_vector)
{
	for(unsigned int i = 0; i < in_vector.size(); i++) {
		if((in_vector.at(i) > 0.000001) || (in_vector.at(i) < -0.000001)) {
			return false;
		}
	}
	return true;
}

bool usingReceivedCartesianVelocity = false;
int main(int argc, char* argv[])
{
//ROS------------------------------------------------------------------------------------------
	yarp::os::Node node1("/yarp/listener");
 	yarp::os::Subscriber<Twist> subscriber;
   
   	 if (!subscriber.topic("/chatter")) {
    	    cerr<< "Failed to subscriber to /chatter\n";
     	    return -1;
   	}
        yarp::os::Node node2("/yarp/talker");

    	yarp::os::Publisher<Twist> publisher;

   	 if (!publisher.topic("/posicion2")) {
    	    cerr<< "Failed to create publisher to /posicion2\n";
    	    return -1;
    	}
        yarp::os::Node node6("/yarp/artvel");
    	yarp::os::Publisher<Twist> publisher2;

   	 if (!publisher2.topic("/artpos")) {
    	    cerr<< "Failed to create publisher to /artpos\n";
    	    return -1;
    	}
        yarp::os::Node node3("/yarp/garra");
 	yarp::os::Subscriber<String> subscriber2;
   
   	 if (!subscriber2.topic("/garra")) {
    	    cerr<< "Failed to subscriber to /garra\n";
     	    return -1;
   	}
        yarp::os::Node node4("/yarp/mode");
 	yarp::os::Subscriber<Bool> subscriber3;
   
   	 if (!subscriber3.topic("/Mode")) {
    	    cerr<< "Failed to subscriber to /Mode\n";
     	    return -1;
   	}
        yarp::os::Node node5("/yarp/return");
 	yarp::os::Subscriber<Bool> subscriber4;
   
   	 if (!subscriber4.topic("/return")) {
    	    cerr<< "Failed to subscriber to /return\n";
     	    return -1;
   	}

//----------------------------------------------------------------------------------------------
	int oldStatus[AMOR_NUM_JOINTS], status[AMOR_NUM_JOINTS];
	int major, minor, build;
	yarp::os::BufferedPort<yarp::os::Bottle> port_dx_i, port_button_i, port_q_o, port_I_o, port_x_o;

	port_dx_i.open("/rate_amor_dx_i");
	port_button_i.open("/rate_amor_button_i");
	port_q_o.open("/rate_amor_q_o");
	port_I_o.open("/leap");
	port_x_o.open("/rate_amor_x_o");

	ttyset();

	amor_get_library_version(&major, &minor, &build);
	printf("AMOR API library version %d.%d.%d\n\n", major, minor, build);

	printf("Press <enter> to issue a controlled stop. Press <esc> to issue an emergency stop/application exit\n\n");

	print_help();

	for(unsigned i = 0; i < AMOR_NUM_JOINTS; ++i)
		g_cartVelocity[i] = 0;

	try
	{
		static const unsigned NUM_CONNECTION_RETRIES = 3;
		unsigned retryCounter = NUM_CONNECTION_RETRIES;
		bool handStopped = true;	
		while(1)
		{
			std::cout << "Trying to connect to AMOR... ";
#ifdef WIN32
			g_hRobot = amor_connect("canlib_esd.dll", 0);
#endif // WIN32
#ifdef UNIX
			g_hRobot = amor_connect("libeddriver.so", 0);
#endif // UNIX
			if(g_hRobot != AMOR_INVALID_HANDLE) // if AMOR connected
				break;
			else
			{
				std::cout << amor_error() << "[FAILED]" << std::endl;
				retryCounter--;
				if(retryCounter == 0)
					throw std::runtime_error("maximum number of connection retries exceeded");

				std::cout << "Press <enter> to retry connecting to AMOR..." << std::endl;
				std::cin.get();
			}
		}

		std::cout << "[OK]" << std::endl;

		for(unsigned i = 0; i < AMOR_NUM_JOINTS; ++i)
		{
			if(amor_get_joint_info(g_hRobot, i, &g_jointInfo[i]))
				throw std::runtime_error(amor_error());

			if(amor_get_status(g_hRobot, i, &oldStatus[i]) != AMOR_SUCCESS)
				throw std::runtime_error(amor_error());
		}

		if(amor_get_actual_positions(g_hRobot, &g_jointPosition) != AMOR_SUCCESS)
			throw std::runtime_error(amor_error());

		unsigned statusCounter = 0;
		char key;

		int zeroRateCounter = 0;
		bool haveStopped = false;
		while(!_abort)
		{

			std::vector<double> cartesianRateReceived;
			for(int i = 0; i < 6; i++) {
				cartesianRateReceived.push_back(0.0);
			}

			// receive Cartesian rate
//-----------------------------------------------------------------------------------------------------
//ROS comunication
			subscriber.read(rdata);
			subscriber3.read(mode);
			subscriber4.read(pose);
			//cout<<"data:"<<rdata.linear.x<<", "<<rdata.linear.y<<", "<<rdata.linear.z<<"."<<endl;
			if (mode.data==false){
				cartesianRateReceived.at(0)=rdata.linear.x;
				cartesianRateReceived.at(1)=rdata.linear.y;
				cartesianRateReceived.at(2)=rdata.linear.z;
				cartesianRateReceived.at(3)=0;
				cartesianRateReceived.at(4)=0;
				cartesianRateReceived.at(5)=0;
				printf("_dx, _dy, _dz: %f, %f, %f.\n", cartesianRateReceived.at(0), cartesianRateReceived.at(1), cartesianRateReceived.at(2));

//-----------------------------------------------------------------------------------------------------

				std::vector<double> enforceCartesianRateReceived;
				for(int i = 0; i < 6; i++) {
					enforceCartesianRateReceived.push_back(0.0);
				}
	
				for(int i = 0; i < 6; i++) {
					 enforceCartesianRateReceived.at(i) = enforceRange(cartesianRateReceived.at(i));
				}
	
				if(checkIfZero(cartesianRateReceived)) {
					zeroRateCounter++;
				}
				else {
					zeroRateCounter = 0;
				}
	
				//if(usingReceivedCartesianVelocity) {
	
					if(zeroRateCounter > 25) {
						if(!haveStopped) {
							printf(" > Receiving none or all zero velocities, commanding stop\n");
							if(amor_controlled_stop(g_hRobot)) {
								HandleError();
							}
							for(int i = 0; i < 7; i++)
							{
								g_jointVelocity[i] = 0.0f;
								g_cartVelocity[i] = 0.0f;
							}
							haveStopped = true;
						}
					}
					else {
	
						g_cartVelocity[0] = - enforceCartesianRateReceived.at(0) * g_translationGain;
						g_cartVelocity[1] = - enforceCartesianRateReceived.at(1) * g_translationGain;
						g_cartVelocity[2] = enforceCartesianRateReceived.at(2) * g_translationGain;
						g_cartVelocity[4] = - enforceCartesianRateReceived.at(4) * g_pitchGain;
						g_cartVelocity[3] = - enforceCartesianRateReceived.at(5) * g_yawGain;
						g_cartVelocity[5] = enforceCartesianRateReceived.at(3) * g_rollGain;


						cout<<"data:  "<<g_cartVelocity[0]<<", "<<g_cartVelocity[1]<<", "<<g_cartVelocity[2]<<", "<<g_cartVelocity[3]<<", "<<g_cartVelocity[4]<<", "<<g_cartVelocity[5]<<"."<<endl;
						if(g_cartVelocity[3]!=0 || g_cartVelocity[4]!=0 || g_cartVelocity[5]!=0){
						cout<<"ERROR\n";
						while(true);
						}
						if(amor_set_cartesian_velocities(g_hRobot,g_cartVelocity) != AMOR_SUCCESS) {
							HandleError();
						}
	
						haveStopped = false;
	
					}
//-----------------------------------------------------------------------------------------------------
//ROS comunication
			}else{
				if(pose.data==false){
				g_jointVelocity[0]=rdata.linear.x;
				g_jointVelocity[1]=0;
				g_jointVelocity[2]=0;
				g_jointVelocity[3]=0;
				g_jointVelocity[4]=0;
				g_jointVelocity[5]=rdata.linear.z;
				g_jointVelocity[6]=0;
				}else{
				g_jointVelocity[0]=0;
				g_jointVelocity[1]=rdata.linear.x;
				g_jointVelocity[2]=rdata.linear.y;
				g_jointVelocity[3]=rdata.linear.z;
				g_jointVelocity[4]=rdata.angular.x;
				g_jointVelocity[5]=rdata.angular.y;
				g_jointVelocity[6]=rdata.angular.z;
				}

				cout<<"vel="<<g_jointVelocity[0]<<", "<<g_jointVelocity[1]<<", "<<g_jointVelocity[2]<<", "<<g_jointVelocity[3]<<", "<<g_jointVelocity[4]<<", "<<g_jointVelocity[5]<<", "<<g_jointVelocity[6]<<endl;

				if(amor_set_velocities(g_hRobot,g_jointVelocity) != AMOR_SUCCESS) {
					HandleError();
				}
			}
//-----------------------------------------------------------------------------------------------------
			//}

			// receive buttons
			yarp::os::Bottle *_bottleReceivedButton = port_button_i.read(false);
			int button1 = 0;
			int button2 = 0;
//-----------------------------------------------------------------------------------------------------
//ROS comunication
			subscriber2.read(garradata);
			cout<<"Recieved:"<<garradata.data<<endl;
			if (garradata.data=="abrir"){
				button1=1;
				button2=0;
			}
			if (garradata.data=="cerrar"){
				button1=0;
				button2=1;
			}
			if (garradata.data!="cerrar" && garradata.data!="abrir"){
				button1=0;
				button2=0;
			}
//-----------------------------------------------------------------------------------------------------
			if (_bottleReceivedButton!=NULL) {
				button1 = (_bottleReceivedButton->find("button1")).asInt();
				button2 = (_bottleReceivedButton->find("button2")).asInt();
			}

			if(button1 == 1) {
				printf(" > Opening hand.\n");
				amor_open_hand(g_hRobot);
				handStopped = false;
			}
			else {
				if(button2 == 1) {
					printf(" > Closing hand.\n");
					amor_close_hand(g_hRobot);
					handStopped = false;
				}
				else {
					if(!handStopped) {
						printf(" > Stopping hand.\n");
						amor_stop_hand(g_hRobot);
						handStopped = true;
					}
				}
			}

			// send joint angles
			AMOR_VECTOR7 jointAngles, jointAnglesDegrees;
			if(amor_get_actual_positions(g_hRobot, &jointAngles) != AMOR_SUCCESS) {
				HandleError();
			}
			for(int i = 0; i < 7; i++) {
				jointAnglesDegrees[i] = jointAngles[i] * 180.0/PI;
			}
			yarp::os::Bottle& bottle_q_o = port_q_o.prepare();
			bottle_q_o.clear();
			bottle_q_o.addString("q1");
			bottle_q_o.addDouble(jointAnglesDegrees[0]);
			bottle_q_o.addString("q2");
			bottle_q_o.addDouble(jointAnglesDegrees[1]);
			bottle_q_o.addString("q2.5");
			bottle_q_o.addDouble(jointAnglesDegrees[2]);
			bottle_q_o.addString("q3");
			bottle_q_o.addDouble(jointAnglesDegrees[3]);
			bottle_q_o.addString("q4");
			bottle_q_o.addDouble(jointAnglesDegrees[4]);
			bottle_q_o.addString("q5");
			bottle_q_o.addDouble(jointAnglesDegrees[5]);
			bottle_q_o.addString("q6");
			bottle_q_o.addDouble(jointAnglesDegrees[6]);
			port_q_o.write();

			// send joint currents
			AMOR_VECTOR7 jointCurrents;
			if(amor_get_actual_currents(g_hRobot, &jointCurrents) != AMOR_SUCCESS) {
				HandleError();
			}
			yarp::os::Bottle& bottle_I_o = port_I_o.prepare();
			bottle_I_o.clear();
			bottle_I_o.addDouble(rdata.linear.x);
			bottle_I_o.addDouble(rdata.linear.y);
			bottle_I_o.addDouble(rdata.linear.z);
			bottle_I_o.addDouble(0);
			bottle_I_o.addDouble(0);
			bottle_I_o.addDouble(0);
			port_I_o.write();

			// get cartesian positions
			AMOR_VECTOR7 cartesianPositions;
			if(amor_get_cartesian_position(g_hRobot, cartesianPositions) != AMOR_SUCCESS) { 
				HandleError();
			}
			yarp::os::Bottle& bottle_x_o = port_x_o.prepare();
			bottle_x_o.clear();
			bottle_x_o.addString("x");
			bottle_x_o.addDouble(MM2M(cartesianPositions[0]));
			bottle_x_o.addString("y");
			bottle_x_o.addDouble(MM2M(cartesianPositions[1]));
			bottle_x_o.addString("z");
			bottle_x_o.addDouble(MM2M(cartesianPositions[2]));
			bottle_x_o.addString("roll");
			bottle_x_o.addDouble(RAD2DEG(cartesianPositions[5]));
			bottle_x_o.addString("pitch");
			bottle_x_o.addDouble(RAD2DEG(cartesianPositions[4]));
			bottle_x_o.addString("yaw");
			bottle_x_o.addDouble(RAD2DEG(cartesianPositions[3]));
			port_x_o.write();
			
//-----------------------------------------------------------------------------------------------------
//ROS comunication
			sdata.linear.x=cartesianPositions[0];
			sdata.linear.y=cartesianPositions[1];
			sdata.linear.z=cartesianPositions[2];
			sdata.angular.x=cartesianPositions[3];
			sdata.angular.y=cartesianPositions[4];
			sdata.angular.z=cartesianPositions[5];
	//		printf("_dx, _dy, _dz: %f, %f, %f.\n",cartesianPositions[0], cartesianPositions[1], cartesianPositions[2]);
			publisher.write(sdata);
			artdata.linear.x=jointAnglesDegrees[1];
			artdata.linear.y=jointAnglesDegrees[2];
			artdata.linear.z=jointAnglesDegrees[3];
			artdata.angular.x=jointAnglesDegrees[4];
			artdata.angular.y=jointAnglesDegrees[5];
			artdata.angular.z=jointAnglesDegrees[6];

			if (artdata.angular.x<0){
				artdata.angular.x=180+(180+artdata.angular.x);
			}
			cout<<"angles="<<artdata.linear.x<<", "<<artdata.linear.y<<", "<<artdata.linear.z<<", "<<artdata.angular.x<<", "<<artdata.angular.y<<", "<<artdata.angular.z<<endl;
			publisher2.write(artdata);
//-----------------------------------------------------------------------------------------------------
			if(KeyboardReadKey(&key))
				ProcessKeyPress(key);

			mssleep(10);

			statusCounter = (statusCounter + 1) & 0x1F; // wrap
			if(statusCounter == 0)
			{
				for(unsigned i = 0; i < AMOR_NUM_JOINTS; ++i)
				{
					if(amor_get_status(g_hRobot, i, &status[i]) != AMOR_SUCCESS)
					{
						HandleError();
						throw std::runtime_error("unable to get AMOR joint status");
					}

					if(status[i] != oldStatus[i])
					{
						if(status[i] < 0)
						{
							const int errorNo = ((-status[i]) >> 16) & 0xF;
							const int errorMotor = ((-status[i]) >> 2) & 0xFF;
							const int errorLowHigh = (-status[i]) & 0x03;
							amor_emergency_stop(g_hRobot);
							std::cout << "ERROR on " << JOINT_NAMES[i] << ": " << std::hex << "0x" << std::setw(6) << std::setfill('0') << -status[i];
							if(errorNo == 6)
								std::cout << "(error no: " << std::dec << errorMotor << "-" << (errorLowHigh == 1 ? "low" : "high") << ")" << std::endl;
							else
								std::cout << std::endl;
							_abort = true;
							break;
						}
						else
							Warning(i,status[i]);
						oldStatus[i] = status[i];
					}
				}
			}
		}
		amor_emergency_stop(g_hRobot);
		amor_release(g_hRobot);
	}
	catch(const std::exception &e)
	{
		std::cout << "exception caught: " << e.what() << std::endl;

		if(AMOR_INVALID_HANDLE != g_hRobot)
			amor_release(g_hRobot);
	}

	std::cout << "Press any key to quit..." << std::endl;
	std::cin.get();

	ttyreset(0);

	return(0);
}

void ProcessKeyPress(char key)
{
	int i;

	switch(key)
	{
	case 27: /* <ESCAPE> key */
		printf(" > Terminating...\n");
		_abort = true;
		break;
	case '?':
		print_help();
		break;
	case 't':
		if(usingReceivedCartesianVelocity){
			printf(" > Toggled Cartesian velocity input OFF\n");
			usingReceivedCartesianVelocity = false;

			printf(" > Stop\n");
			if(amor_controlled_stop(g_hRobot))
				HandleError();
			for(i = 0; i < 7; i++)
			{
				g_jointVelocity[i] = 0.0f;
				g_cartVelocity[i] = 0.0f;
			}
			break;
		}
		else {
			printf(" > Toggled Cartesian velocity input ON\n");
			usingReceivedCartesianVelocity = true;
		}
	break;
	case 'i': // initial position
		{
			printf(" > Toggled Cartesian velocity input OFF\n");
			usingReceivedCartesianVelocity = false;

			static char _c = 'n';
			while(true) {
				printf(" > Enter 'y' to confirm moving to initial position.\n");
				printf(" > WARNING: Make sure robot path is free of obstacles!\n");
				scanf ("%s",&_c);
				if(_c == 'y') {

					printf(" > Initial   : initial pose\n");
					g_jointPosition[0] = INITIAL_POSITION[0];
					g_jointPosition[1] = INITIAL_POSITION[1];
					g_jointPosition[2] = INITIAL_POSITION[2];
					g_jointPosition[3] = INITIAL_POSITION[3];
					g_jointPosition[4] = INITIAL_POSITION[4];
					g_jointPosition[5] = INITIAL_POSITION[5];
					g_jointPosition[6] = INITIAL_POSITION[6];
					if(amor_set_positions(g_hRobot,g_jointPosition) != AMOR_SUCCESS)
						HandleError();

					break;
				}
				else {

					printf(" > Aborted move to initial position...\n");
					break;
				}
			}

		}
		break;
	default:
		printf(" > Toggled Cartesian velocity input OFF\n");
		usingReceivedCartesianVelocity = false;

		printf(" > Stop\n");
		if(amor_controlled_stop(g_hRobot))
			HandleError();
		for(i = 0; i < 7; i++)
		{
			g_jointVelocity[i] = 0.0f;
			g_cartVelocity[i] = 0.0f;
		}
		break;
	}
}

void print_help(void)
{
	printf("'?'   Help                   \n");
	printf("'i'   Go to initial pose     \n");
	printf("'t'   Toggle Cartesian input \n");
	printf("'Esc' Quit                   \n");
	printf(" press any other key to stop all movements!\n");
	print_separator();
}

void print_separator(void)
{
	printf("------------------------------------------------------------\n");
}

/* sleep for the provided amount of milliseconds */
void mssleep(int ms)
{
#ifdef WIN32
	Sleep(ms);
#endif // WIN32
#ifdef UNIX
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = (ms * 1000);
	select(0, NULL, NULL, NULL, &tv);
#endif // UNIX
}

/* configure the TTY (only in UNIX) for reading keyboard input */
void ttyset(void)
{
#ifdef UNIX
	struct termios ts;
	struct sigaction sact;
	tcgetattr(STDIN_FILENO, &ts);
	ots = ts;
	ts.c_lflag &= ~ICANON; /* raw data mode */
	ts.c_lflag &= ~(ECHO | ECHOCTL | ECHONL); /* no echo */
	ts.c_lflag |= IEXTEN;

	/* restore tty after these signals */
	sact.sa_handler = ttyreset;
	sigaction(SIGHUP, &sact, NULL);
	sigaction(SIGINT, &sact, NULL);
	sigaction(SIGPIPE, &sact, NULL);
	sigaction(SIGTERM, &sact, NULL);
	tcsetattr(STDIN_FILENO, TCSANOW, &ts); /* set raw data mode */
	fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK); /* make stdin non blocking */
	fcntl(STDOUT_FILENO, F_SETFL, fcntl(STDOUT_FILENO, F_GETFL, 0) | O_NONBLOCK); /* make stdout non blocking */
#endif // UNIX
}

/* resets the TTY configurations (only in UNIX) that was changed in the ttyset function */
void ttyreset(int signal)
{
#ifdef UNIX
	tcsetattr(STDIN_FILENO, TCSANOW, &ots);
	tcsetattr(STDOUT_FILENO, TCSANOW, &ots);
#endif // UNIX
	exit(signal);
}

double enforceRange(double in)
{

	double out;

	if(in > 1.0)
	{
		out = 1.0;
	}
	else
	{
		if(in < -1.0)
		{
			out = -1.0;
		}
		else
		{
			out = in;
		}
	}
	return out;

}




