#include "ros/ros.h"
#include "quadrotor_control/manipulated_variables.h"
#include "std_srvs/Empty.h"

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include "Dynamics.h"

double X[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
double X_dot[12];
double m;
double g;
double U[7] = {0, 0, 0, 0, 0, 0, 0};
double C[6] = {0, 0, 0, 0, 0, 0};
tf::Matrix3x3 A_T;

void callback_manipulated_variables( const quadrotor_control::manipulated_variables::Ptr& msg )
{    
  ROS_INFO("Dynamics: Callback_manipulated_variables");
  U[0] = msg->U[0];
  U[1] = msg->U[1];
  U[2] = msg->U[2];  
  U[3] = msg->U[3];
}

tf::Vector3 calcWindInfluence( ){
  tf::Vector3 deltaV = tf::Vector3( X[3] - U[4], X[4] - U[5], X[5] - U[6]);
  tf::Matrix3x3 R_NB;
  R_NB.setEulerYPR( X[6], X[7], X[8] );
  // es gibt timesTranspose(Matrix) und TransposeTimes(Matrix) -> nochmal checken
  tf::Matrix3x3 R_BN = R_NB.transpose();
  return tf::Vector3(( deltaV.length()/m ) * (R_NB * A_T * R_BN * deltaV));
}

void Berechne_Zustandsgroessen(){
  X_dot[0] = X[3];
  X_dot[1] = X[4]; 
  X_dot[2] = X[5];  
  tf::Vector3 windInfluence = calcWindInfluence();
  double a1 = cos(X[6])*sin(X[7])*cos(X[8]);
  double a2 = sin(X[6])*sin(X[8]);
  double a3 = cos(X[6])*sin(X[7])*sin(X[8]);
  double a4 = cos(X[6])*cos(X[7]);
  double c1 = U[0]/m;
  X_dot[3] =   -c1*( a1+a2 ) - windInfluence.getX();
  X_dot[4] =   -c1*( a3-a2 ) - windInfluence.getY();
  X_dot[5] = g -c1*( a4 )    - windInfluence.getZ();
  X_dot[6] = X[9];
  X_dot[7] = X[10];
  X_dot[8] = X[11];
  X_dot[9] = X[10]*X[11]*C[0] + C[1]*U[1];
  X_dot[10]= X[9] *X[11]*C[2] + C[3]*U[2];
  X_dot[11]= X[9] *X[10]*C[4] + C[5]*U[3];
  ROS_INFO("Dynamics: Berechne Zustandsgroessen");
}

bool propagate( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
   Berechne_Zustandsgroessen();
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
   
   ros::ServiceServer service = nh.advertiseService("dynamics_prop", propagate);
   ROS_INFO( "Dynamics Init done" );
   ros::spin();
   return 0;
}

