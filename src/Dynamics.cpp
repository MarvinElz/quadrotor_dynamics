#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <quadrotor_dynamics/WindConfig.h>

#include "quadrotor_control/manipulated_variables.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

//#define DIRECT_LOOPBACK

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "quadrotor_control/pose.h"

// #define LOGGING
#ifdef LOGGING
	#include <fstream>
	using namespace std;
	ofstream logFile;
	double simTime = 0;
#endif

#include "quadrotor_control/kinematics.h"

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include "Dynamics.h"

// Zustandsvektor
double X[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Masse des UAV
double m;

// Gravitationskonstante
double g;

// Eingangsvector
double U[7] = {7, 0, 0, 0, 0, 2, 0};

// UAV-Konstanten ( Trägheiten )
double C[6] = {0, 0, 0, 0, 0, 0};

// Einfluss der Windgeschwindigkeit auf Bewegung
tf::Matrix3x3 A_T;

// Zeitpunkt der letzten Propagation ( Berechnung der Schrittweite )
ros::Time last_Prop; 

// Publisher für Pose- und Geschwindigkeitsdaten
ros::Publisher pub_Kin;


/*
	Servicefunktion für dynamische Paramter zum Ändern der Windrichtung/-geschwindigkeit
*/
void callbackSetWind(quadrotor_dynamics::WindConfig &config, uint32_t level) {	
	U[4] = config.WindX; 
	U[5] = config.WindY; 
	U[6] = config.WindZ;
	ROS_INFO( "Wind-Direction: %f, %f, %f", U[4], U[5], U[6] );
}

/*
	Subscribefunktion für Stellgrößen aus der Regelung
	-> lokale Speicherung der Stellgrößen
*/
void callback_manipulated_variables( const quadrotor_control::manipulated_variables::Ptr& msg )
{   
  	U[0] = msg->U[0];
  	U[1] = msg->U[1];
  	U[2] = msg->U[2];  
  	U[3] = msg->U[3];
}

/*
	Berechnung des Einflusses der Luftreibung auf Bewegung
	Paramter:
		- die aktuelle Pose und Geschwindigkeiten des UAV
		- die Windrichtung und -geschwindigkeit
*/
tf::Vector3 calcWindInfluence( const double *X, const double *U ){

	// Geschwindigkeitsdifferenz UAV <-> Wind (im N-System)
 	tf::Vector3 deltaV = tf::Vector3( X[3] - U[4], X[4] - U[5], X[5] - U[6]);

	// Transformationsmatrix von B -> N
 	tf::Matrix3x3 R_NB;
 	R_NB.setEulerYPR( X[8], X[7], X[6] );

 	// es gibt timesTranspose(Matrix) und TransposeTimes(Matrix) -> nochmal checken
	// Transformationsmatrix von N -> B ( Transponierte von B -> N )
 	tf::Matrix3x3 R_BN = R_NB.transpose();

	// Berechnung des Windeinflusses im B-System
	// danach Überführung in N-System ( da restliche Simulation im N-System )	
 	tf::Vector3 windInfl = ( deltaV.length()/m ) * (R_NB * A_T * R_BN * deltaV);
 	return windInfl;
}

/*
	Berechnung der Funktion f(X,U) = X'
  (nähere Beschreibung siehe Kapitel 2.3, 2.4)
*/
void Berechne_Zustandsgroessen(double *X_dot){
	// Änderung der Position
	X_dot[0] = X[3];
  X_dot[1] = X[4]; 
  X_dot[2] = X[5];  

  tf::Vector3 windInfluence = calcWindInfluence( X, U );

  double a1 = cos(X[6])*sin(X[7])*cos(X[8]);
  double a2 = sin(X[6])*sin(X[8]);
  double a3 = cos(X[6])*sin(X[7])*sin(X[8]);
  double a4 = sin(X[6])*cos(X[8]);
  double a5 = cos(X[6])*cos(X[7]);
  double c1 = U[0]/m;

	// Änderung der Geschwindigkeit
  X_dot[3] =   -c1*( a1+a2 ) - windInfluence.getX();
  X_dot[4] =   -c1*( a3-a4 ) - windInfluence.getY();
  X_dot[5] = g -c1*( a5 )    - windInfluence.getZ();

	// Änderung der Orientierung
  X_dot[6] = X[9];
  X_dot[7] = X[10];
  X_dot[8] = X[11];

	// Änderung der Winkelgeschwindigkeit
  X_dot[9] = X[10]*X[11]*C[0] + C[1]*U[1];
  X_dot[10]= X[9] *X[11]*C[2] + C[3]*U[2];
  X_dot[11]= X[9] *X[10]*C[4] + C[5]*U[3];  
}

/*
	Integrationsalgorithmus: Euler
*/
void Integrate( const double dt, const double *X_dot ){
	
  X[0] 	+= dt * X_dot[0];
  X[1] 	+= dt * X_dot[1];
  X[2] 	+= dt * X_dot[2];

  X[3] 	+= dt * X_dot[3];
  X[4] 	+= dt * X_dot[4];
  X[5] 	+= dt * X_dot[5];
  
  X[6] 	+= dt * X_dot[6];
  X[7]	+= dt * X_dot[7];
  X[8] 	+= dt * X_dot[8];

  X[9] 	+= dt * X_dot[9];
  X[10] += dt * X_dot[10];
  X[11] += dt * X_dot[11];
}

/*
	Berechnung der Funktion g(X,U) = Y
	(nähere Beschreibung siehe Kapitel 2.3, 2.4)
	Integration über 
*/
void Berechne_Ausgangsgroessen( double *Y ){
	for( int i = 0; i < 12; i++ )
		Y[i] = X[i];
}

/*
	Service zum Zurücksetzen der UAV-Kinematikdaten
	auf einen sicheren Zustand ( zum Kollosionsschutz des youBots )
*/
bool resetPosition( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
	ROS_INFO("Reset Position");
	for(int i = 0; i < 12; i++)
		X[i] = 0;
	// Achtung: abhängig von Skalierung in quadrotor_youbot_interface
	X[2] = -26.34;

	quadrotor_control::kinematics k;
 	k.pose.position.x = X[0];
 	k.pose.position.y = X[1];
 	k.pose.position.z = X[2];
	
 	k.vel.linear.x = X[3];
 	k.vel.linear.y = X[4];
 	k.vel.linear.z = X[5];

 	k.pose.orientation.x = X[6];
 	k.pose.orientation.y = X[7];
 	k.pose.orientation.z = X[8];

 	k.vel.angular.x= X[9];
 	k.vel.angular.y= X[10];
 	k.vel.angular.z= X[11];
 	pub_Kin.publish(k);
	return true;
}

/*
	Bereitgestellte Servicefunktion:
		berechnet UAV-Dynamik
*/
bool propagate( std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp )
{
	double X_dot[12];

	// wird die Simulation nach einer Pause erneut gestartet
	// muss last_Prop auf aktuelle Zeit angepasst werden,
	// damit Schrittweite nicht zu groß wird
	if( req.data )
		last_Prop = ros::Time::now();

	ros::Time now = ros::Time::now();
  double dt = (now - last_Prop).toSec();

	// konstante Schrittweite für Testzwecke
	dt = 0.005;								
	
	Berechne_Zustandsgroessen( X_dot );
	
	Integrate( dt, X_dot );

	double Y[12];
	Berechne_Ausgangsgroessen( Y );  

  ROS_INFO( "Vx: %f, Vy: %f, Vz: %f, VPsi: %f, Z: %f, Phi: %f, Theta: %f, Psi: %f", Y[3], Y[4], Y[5], Y[11], Y[2], Y[6], Y[7], Y[8] );
	
	#ifdef LOGGING
		simTime += dt;
		logFile << simTime << "," << Y[3] << "," << Y[5] << "," << Y[11] << std::endl; 
	#endif
	
	// last_Prop auf aktuelle Zeit setzen
	// (Vorbereitung für nächste Iteration) 
	last_Prop = now;

	quadrotor_control::kinematics k;
 	k.pose.position.x = Y[0];
 	k.pose.position.y = Y[1];
 	k.pose.position.z = Y[2];
	
 	k.vel.linear.x = Y[3];
 	k.vel.linear.y = Y[4];
 	k.vel.linear.z = Y[5];

 	k.pose.orientation.x = Y[6];
 	k.pose.orientation.y = Y[7];
 	k.pose.orientation.z = Y[8];

 	k.vel.angular.x= Y[9];
 	k.vel.angular.y= Y[10];
 	k.vel.angular.z= Y[11];
 	pub_Kin.publish(k);

 	return true;
}

/*
	laden der Paramter vom Parameterserver
*/
void calcConstants(const ros::NodeHandle &nh){
  	double Ix, Iy, Iz, L;
  	nh.getParam("m", m);
  	nh.getParam("g", g);
  	nh.getParam("Ix", Ix);
  	nh.getParam("Iy", Iy);
  	nh.getParam("Iz", Iz);
  	nh.getParam("L" , L);
  	C[0] = (Iy - Iz)/Ix;
  	C[1] = L/Ix;
  	C[2] = (Iz - Ix)/Iy;
  	C[3] = L/Iy;
  	C[4] = (Ix - Iy)/Iz;
  	C[5] = 1/Iz;
  	double Axx, Ayy, Azz;
  	nh.getParam("Axx", Axx);
  	nh.getParam("Ayy", Ayy);
  	nh.getParam("Azz", Azz); 
  	A_T = tf::Matrix3x3( Axx, 0, 0, 0, Ayy, 0, 0, 0, Azz );
}

int main( int argc, char * argv[] ){

	ros::init(argc, argv, "Dynamics");
	ros::NodeHandle nh("Dynamics");
	calcConstants(nh);

	// Vorbereitung für Simulation
	last_Prop = ros::Time::now();   

	#ifdef LOGGING
		char filePathName[] = "/home/student/Schreibtisch/log.txt";
		logFile.open(filePathName); 
		if(!logFile.is_open()){
			ROS_ERROR("Logfile: '%s' konnte nicht geöffnet werden. Beende.", filePathName);
			return 0;
		}
	#endif

	// Einstellung des Windes über dynamische Parameter
	dynamic_reconfigure::Server<quadrotor_dynamics::WindConfig> server;
	dynamic_reconfigure::Server<quadrotor_dynamics::WindConfig>::CallbackType f;
	f = boost::bind(&callbackSetWind, _1, _2);
	server.setCallback(f);

	// Subscriber für Stellgrößen aus der Regelung
	ros::Subscriber sub_stell = nh.subscribe("/stellgroessen", 10, callback_manipulated_variables);

	// für Testzwecke wurden die Kinematikdaten direkt an die Regelung zurückgegeben (/kin_measure)
	// für HIL müssen diese an den youBot gesendet werden (/kin_model)
	#ifdef DIRECT_LOOPBACK
		ROS_INFO("Nutze direkte Rückführung an Regelung");
		pub_Kin = nh.advertise<quadrotor_control::kinematics>("/kin_measure", 10);
	#else
		ROS_INFO("Nutze youbot");
		pub_Kin = nh.advertise<quadrotor_control::kinematics>("/kin_model", 10);
	#endif
   
	// Bereitstellung des Services dynamics_prop zum Ausführen der Propagation
	ros::ServiceServer srv_prop = nh.advertiseService("dynamics_prop", propagate);

	// Bereitstellung des Services dynamics_resetPosition zum Zurücksetzen der UAV-Pose 
	// an einen (für den youBot) sicheren Wert -> Kollisionsschutz
	ros::ServiceServer srv_reset = nh.advertiseService("dynamics_resetPosition", resetPosition);
	ROS_INFO( "Dynamics Init done" );

	ros::spin();

	#ifdef LOGGING
	logFile.close();
	#endif
	return 0;
}



