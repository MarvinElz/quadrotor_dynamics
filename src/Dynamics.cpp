#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <quadrotor_dynamics/WindConfig.h>

#include "quadrotor_control/manipulated_variables.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#define DIRECT_LOOPBACK
#ifdef DIRECT_LOOPBACK
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "quadrotor_control/pose.h"
#endif

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include "Dynamics.h"

double X[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//double X_dot[12];
double m;
double g;
double U[7] = {7, 0, 0, 0, 0, 2, 0};
double C[6] = {0, 0, 0, 0, 0, 0};
tf::Matrix3x3 A_T;
ros::Time last_Prop; 

#ifdef DIRECT_LOOPBACK
ros::Publisher pub_Vel;
ros::Publisher pub_Pos;
#endif

void callbackSetWind(quadrotor_dynamics::WindConfig &config, uint32_t level) {	
	U[4] = config.WindX; 
	U[5] = config.WindY; 
	U[6] = config.WindZ;
	ROS_INFO( "Wind-Direction: %f, %f, %f", U[4], U[5], U[6] );
}

void callback_manipulated_variables( const quadrotor_control::manipulated_variables::Ptr& msg )
{    
  	//ROS_INFO("Dynamics: Callback_manipulated_variables");
  	U[0] = msg->U[0];
  	U[1] = msg->U[1];
  	U[2] = msg->U[2];  
  	U[3] = msg->U[3];
}

tf::Vector3 calcWindInfluence( const double *X, const double *U ){
  	tf::Vector3 deltaV = tf::Vector3( X[3] - U[4], X[4] - U[5], X[5] - U[6]);
  	tf::Matrix3x3 R_NB;
  	R_NB.setEulerYPR( X[8], X[7], X[6] );
  	// es gibt timesTranspose(Matrix) und TransposeTimes(Matrix) -> nochmal checken
  	tf::Matrix3x3 R_BN = R_NB.transpose();
  	tf::Vector3 windInfl = ( deltaV.length()/m ) * (R_NB * A_T * R_BN * deltaV);
  	//ROS_INFO("WindInfluence: %f, %f, %f", windInfl.getX(), windInfl.getY(), windInfl.getZ());
  	return windInfl;
}

void Berechne_Zustandsgroessen(double *X_dot){
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
  	X_dot[3] =   -c1*( a1+a2 ) - windInfluence.getX();
  	X_dot[4] =   -c1*( a3-a4 ) - windInfluence.getY();
  	X_dot[5] = g -c1*( a5 )    - windInfluence.getZ();
  	X_dot[6] = X[9];
  	X_dot[7] = X[10];
  	X_dot[8] = X[11];
  	X_dot[9] = X[10]*X[11]*C[0] + C[1]*U[1];
  	X_dot[10]= X[9] *X[11]*C[2] + C[3]*U[2];
  	X_dot[11]= X[9] *X[10]*C[4] + C[5]*U[3];  
}

void Berechne_Ausgangsgroessen( const double dt, const double *X_dot ){
  //ROS_INFO( "Dynamics: dt = %f", dt );
  //ROS_INFO( "Ax: %f, Ay: %f, Az: %f", X_dot[3], X_dot[4], X_dot[5] );
  X[0] += dt * X_dot[0];
  X[1] += dt * X_dot[1];
  X[2] += dt * X_dot[2];
  X[3] += dt * X_dot[3];
  X[4] += dt * X_dot[4];
  X[5] += dt * X_dot[5];
  
  X[6] += dt * X_dot[6];
  X[7] += dt * X_dot[7];
  X[8] += dt * X_dot[8];
  X[9] += dt * X_dot[9];
  X[10] += dt * X_dot[10];
  X[11] += dt * X_dot[11];
  ROS_INFO( "Vx: %f, Vy: %f, Vz: %f, VPsi: %f, Phi: %f, Theta: %f, Psi: %f", X[3], X[4], X[5], X[11], X[6], X[7], X[8] );
}



bool propagate( std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp )
{
	double X_dot[12];
	if( req.data )
		last_Prop = ros::Time::now();

   	Berechne_Zustandsgroessen( X_dot );
   	ros::Time now = ros::Time::now();
   	ros::Duration dt = now - last_Prop;
   	Berechne_Ausgangsgroessen(dt.toSec(), X_dot);   
   	last_Prop = now;
   	//ROS_INFO( "Position: %f, %f, %f", X[0], X[1], X[2] );

   	#ifdef DIRECT_LOOPBACK
      	geometry_msgs::Twist vel;
      	vel.linear.x = X[3];
      	vel.linear.y = X[4];
      	vel.linear.z = X[5];
      	vel.angular.x= X[9];
      	vel.angular.y= X[10];
      	vel.angular.z= X[11];
      	pub_Vel.publish(vel);

      	quadrotor_control::pose pos;
      	pos.position.x = X[0];
      	pos.position.y = X[1];
      	pos.position.z = X[2];
      	pos.orientation.x = X[6];
      	pos.orientation.y = X[7];
      	pos.orientation.z = X[8];
      	pub_Pos.publish(pos);
   	#endif
   	return true;
}

void calcConstants(ros::NodeHandle &nh){
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
   	last_Prop = ros::Time::now();   

	dynamic_reconfigure::Server<quadrotor_dynamics::WindConfig> server;
	dynamic_reconfigure::Server<quadrotor_dynamics::WindConfig>::CallbackType f;
	f = boost::bind(&callbackSetWind, _1, _2);
	server.setCallback(f);

   	ros::Subscriber sub_stell = nh.subscribe("/stellgroessen", 10, callback_manipulated_variables);

   	#ifdef DIRECT_LOOPBACK
      	pub_Vel = nh.advertise<geometry_msgs::Twist>("/vel_measure", 10);
      	pub_Pos = nh.advertise<quadrotor_control::pose>("/pose_measure", 10); 
   	#endif
   
   	ros::ServiceServer service = nh.advertiseService("dynamics_prop", propagate);
   	ROS_INFO( "Dynamics Init done" );

   	ros::spin();

   return 0;
}

