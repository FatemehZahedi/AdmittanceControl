#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>
#include "UdpServer.h"


using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;

/* IP Address/Port for KONI Connection */
#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"

/* GUI UDP Server Address/Port */
const std::string 	udp_addr_gui("192.168.0.103");
const int 			udp_port_gui = 50000;

FILE *NewDataFile(void);


/* Shared Memory Function Prototype */
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements);



// Main
int main(int argc, char** argv)
{
	// UDP Server address and port hardcoded now -- change later
	UDPServer udp_server(udp_addr_gui, udp_port_gui);

	/* Force Related Varibles */
	double ftx;						// Force x-direction (filtered)
	double fty;						// Force y-direction (filtered)
	double ftx_un;					// Force x-direction (unfiltered)
	double fty_un;					// Force y-direction (unfiltered)
	double zerox = 0;				// Force x-baseline
	double zeroy = 0;				// Force y-baseline
	double ftx_0 = 0.0;				// Part of force filtering calc
	double fty_0 = 0.0;  			// Part of force filtering calc
	double al = 0.5;				// exponential moving average alpha level

	/* Shared Memory Setup */
	std::string shmAddr("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile");
	int shmnElements = 2;
	int * data = InitSharedMemory<int>(shmAddr, shmnElements);

	/* Force baseline variables/flags */
	int firstIt = 0;			// first iteration flag
	int steady = 0;				// Flag to collect average first 2000 samples of forces without moving KUKA

	/* Euler Angles */
	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;


	// ----------------------Initial DH Parameters------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;



	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;					// calculated joint space parameter differences (between current and prev iteration)
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;		// (current - initial) joint space parameters
	MatrixXd q_init(6, 1); q_init << 0, 0, 0, 0, 0, 0;			// initial joint space parameter (after steady parameter is met)

	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;				// end effector equilibrium position
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;			// force vector (filtered)
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;			// joint space parameters
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;			// calculated current position and pose
	MatrixXd xdot(6, 1);  xdot << 0, 0, 0, 0, 0, 0;				// first derative of x_new
	MatrixXd xdotdot(6, 1); xdotdot << 0, 0, 0, 0, 0, 0;		// second derativive of x_new

	MatrixXd x_old(6, 1); x_old << 0, 0, 0, 0, 0, 0;			// one iteration old position and pose
	MatrixXd x_oldold(6, 1); x_oldold << 0, 0, 0, 0, 0, 0;		// two iteration old position and pose

	double xy_coord[2];											// xy coordinates that are send to gui


	// parse command line arguments
	const char* hostname = (argc >= 3) ? argv[2] : DEFAULT_IP; //optional command line argument for ip address (default is for KONI)
	int port = (argc >= 4) ? atoi(argv[3]) : DEFAULT_PORTID; //optional comand line argument for port


	int count = 0;					// iteration counter
	float sampletime = 0.001;
	double MJoint[7] = { 0 };		// measured joint position

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };	// will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits in radians (can be changed to further restrict motion of robot)


	//calculate max step value
	for (int i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}

	// create new joint position client
	PositionControlClient client;
	client.InitValues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);

	// create new udp connection for FRI
	UdpConnection connection;

	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);


	//create file for output
	FILE * OutputFile = NewDataFile();


	// Initial Joint Angles
	client.NextJoint[0] = -1.5708;
	client.NextJoint[1] = 1.5708;
	client.NextJoint[2] = 0;
	client.NextJoint[3] = 1.5708;
	client.NextJoint[4] = 0;
	client.NextJoint[5] = -1.5708;
	client.NextJoint[6] = -0.958709;
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));



	while (true)
	{

		app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0
			if (count == 1)//first time inside
			{
				sampletime = client.GetTimeStep();
				//calculate max step value
				for (int i = 0; i < 7; i++)
				{
					client.MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
				}

			}

			// Update measured joint angle values
			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);

			// Forward Kinematic
			theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

			MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
									 sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
									 0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
									 0, 0, 0, 1;
			MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
									 sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
									 0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
									 0, 0, 0, 1;
			MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
									 sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
									 0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
									 0, 0, 0, 1;
			MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
									 sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
									 0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
									 0, 0, 0, 1;
			MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
									 sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
									 0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
									 0, 0, 0, 1;
			MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
									 sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
									 0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
									 0, 0, 0, 1;

			MatrixXd T01(4, 4); T01 << A1;
			MatrixXd T02(4, 4); T02 << T01*A2;
			MatrixXd T03(4, 4); T03 << T02*A3;
			MatrixXd T04(4, 4); T04 << T03*A4;
			MatrixXd T05(4, 4); T05 << T04*A5;
			MatrixXd T06(4, 4); T06 << T05*A6;

			// Inverse Kinematic
			phi_euler = atan2(T06(1, 2), T06(0, 2));
			theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
			psi_euler = atan2(T06(2, 1), -T06(2, 0));

			MatrixXd z0(3, 1); z0 << 0, 0, 1;
			MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
			MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
			MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
			MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
			MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
			MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

			MatrixXd p0(3, 1); p0 << 0, 0, 0;
			MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
			MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
			MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
			MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
			MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
			MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

			MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
									-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0), z0(1, 0), z0(2, 0);
			MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
									-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0), z1(1, 0), z1(2, 0);
			MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
									-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0), z2(1, 0), z2(2, 0);
			MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
									-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0), z3(1, 0), z3(2, 0);
			MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
									-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0), z4(1, 0), z4(2, 0);
			MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
									-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0), z5(1, 0), z5(2, 0);


			MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6; 	// Geometric Jacobian
			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
										 0, 1, 0, 0, 0, 0,
										 0, 0, 1, 0, 0, 0,
										 0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
										 0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
										 0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;			// Analytical Jacobian

			// Initializing Stiffness Damping and Inertia
			MatrixXd stiffness(6, 6); stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
												   0, 10000000, 0, 0, 0, 0, //up
												   0, 0, 0, 0, 0, 0, //out toward workshop
												   0, 0, 0, 1000000, 0, 0,
												   0, 0, 0, 0, 1000000, 0,
												   0, 0, 0, 0, 0, 1000000;

			MatrixXd damping(6, 6); damping << 30, 0, 0, 0, 0, 0,
												0, 100, 0, 0, 0, 0,
												0, 0, 30, 0, 0, 0,
												0, 0, 0, 0.5, 0, 0,
												0, 0, 0, 0, 0.5, 0,
												0, 0, 0, 0, 0, 0.5;

			MatrixXd inertia(6, 6); inertia << 7, 0, 0, 0, 0, 0,
											   0, 0.000001, 0, 0, 0, 0,
											   0, 0, 10, 0, 0, 0,
											   0, 0, 0, 0.0001, 0, 0,
											   0, 0, 0, 0, 0.0001, 0,
											   0, 0, 0, 0, 0, 0.0001;


			if (firstIt == 0)	//first time inside
			{
				firstIt = 1;
				// Initialize equilibrium position and pose
				x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
				x_old << x_e;
				x_oldold << x_e;
				x_new << x_e;
			}

			// Get force data from shared memory, convert to Newtons
			ftx = (double)data[0] / 1000000 - zerox; //toward varun desk
			ftx_un = (double)data[0] / 1000000 - zerox;

			fty = (double)data[1] / 1000000 - zeroy;
			fty_un = (double)data[1] / 1000000 - zeroy;

			// Filter force data with exponential moving average
			ftx = al*ftx + (1 - al)*ftx_0;
			ftx_0 = ftx;
			fty = al*fty + (1 - al)*fty_0;
			fty_0 = fty;

			force << ftx,0,fty,0,0,0;


			/* For first 2 seconds, apply exponential moving average to force
			measurements, this will be the bias value */
			steady = steady + 1;
			if (steady < 2000)
			{
			    force << 0, 0, 0, 0, 0, 0;

			    zerox = al*(double)data[0] / 1000000 + (1 - al)*zerox;
			    zeroy = al*(double)data[1] / 1000000 + (1 - al)*zeroy;

			    q_new(0) = -1.5708;
			    q_new(1) = 1.5708;
			    q_new(2) = 0;
			    q_new(3) = 1.5708;
			    q_new(4) = 0;
			    q_new(5) = -1.5708;
			    q_init << q_new;
			 }

			fprintf(OutputFile, "%d %lf %lf %lf %lf %lf %lf %lf %lf %1f %lf %lf %lf %lf %lf %lf\n", count, MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6], force(0), force(2), x_new(0), x_new(1), x_new(2), x_new(3), x_new(4), x_new(5));

			// Shift old position/pose vectors, calculate new
			x_oldold << x_old;
			x_old << x_new;
			x_new << (inertia/(0.000001) + damping/(0.001) + stiffness).inverse()*(force + (inertia/(0.000001))*(x_old - x_oldold) + stiffness*(x_e - x_old)) + x_old;

			// Implement virtual wall
			if (steady > 2000)
			{
			    if (x_new(2) >= 0.94)
			    {
			      x_new(2) = 0.94;
			    }

			    if (x_new(2) <= 0.58)
			    {
			      x_new(2) = 0.58;
			    }

			    if (x_new(0) >= 0.18)
			    {
			      x_new(0) = 0.18;
			    }

			    if (x_new(0) <= -0.18)
			    {
			      x_new(0) = -0.18;
			    }
			}

			/* Calculate new joint velocities/angles */
			if (0.58 <= x_new(2) & x_new(2) <= 0.94)
			{
			    if (-0.18 <= x_new(0) & x_new(0) <= 0.18)
			    {
			      qc << Ja.inverse()*(x_new - x_old);
			      delta_q << delta_q +qc;
			      q_new << delta_q +q_init;
			    }
			}

			// Register new joint angles with KUKA
			client.NextJoint[0] = q_new(0);
			client.NextJoint[1] = q_new(1);
			client.NextJoint[2] = q_new(2);
			client.NextJoint[3] = q_new(3);
			client.NextJoint[4] = q_new(4);
			client.NextJoint[5] = q_new(5);
			client.NextJoint[6] = -0.958709;

			// Send data to visualizer gui
			xy_coord[0] = T06(0,3);
			xy_coord[1] = T06(2,3);
			udp_server.Send(xy_coord, 2);
		}
	}

	// Dead code but useful (maybe in future)
	fclose(OutputFile);
	fprintf(stdout, "File closed.\n\n\n");
	usleep(10000000);//microseconds //wait for close on other side

	// disconnect from controller
	app.disconnect();

	return 1;
}



FILE *NewDataFile(void) //this may be specific to linux OS
{
	FILE *fp;
	time_t rawtime;
	struct tm *timeinfo;
	char namer[40];

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(namer, 40, "Output%Y-%j-%H_%M_%S.txt", timeinfo);//creates a file name that has year-julian day-hour min second (unique for each run, no chance of recording over previous data)
	fp = fopen(namer, "w");//open output file
	return fp;
}


// Shared Memory-------------------------------------------------------
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements){
	key_t key;
	int shmid;
	size_t shmSize = nElements*sizeof(T);
	T * shm = (T *) malloc(shmSize);
	/* make the key */
	if ((key = ftok(shmAddr.c_str(), 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, shmSize, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	shm = (T *) shmat(shmid, (void *)0, 0);

	if (shm == (T *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<nElements; i++)
	{
		shm[i] = 0.0;
	}

	return shm;
}
