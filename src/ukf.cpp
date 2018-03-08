#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

   // set is_initialized_ variable
  is_initialized_=false;	
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  //State Dimension	
  n_x_=x_.size();
  
  //Augmented state estimation
  n_aug_=n_x_+2;
  
  //Sigma points 
  n_sig_=2*n_aug_+1;
  
 //Sigma points prediction dimension
  Xsig_pred_=MatrixXd(n_x_,n_sig_);
 
 //sigma point spreading parameter
 lambda_=3-n_aug_;

 //weights for sigma points
 weights_=VectorXd(n_sig_);

 //Radar noise matrix

 R_radar_=MatrixXd(3,3);
 R_radar_<<std_radr_*std_radr_,0,0,
	   0,std_radphi_*std_radphi_,0,
	   0,0,std_radrd_*std_radrd_;

 R_lidar_=MatrixXd(2,2);
 R_lidar_<<std_laspx_*std_laspx_,0,
	  0,std_laspy_*std_laspy_;
   

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
if(!is_initialized_)
{
  P_<<1,0,0,0,0,
      0,1,0,0,0,
      0,0,1,0,0,
      0,0,0,1,0,
      0,0,0,0,1;

   if(meas_package.sensor_type_==MeasurementPackage::RADAR)
   {
    // convert radar from polar to cartesian and initialize state
  	float rho = meas_package.raw_measurements_[0]; 
  	float phi = meas_package.raw_measurements_[1];
  	float rho_dot = meas_package.raw_measurements_[2];

  	float px = phi * cos(phi);
  	float py = rho * sin(phi);
  	float vx = rho_dot * cos(phi);
  	float vy = rho_dot * sin(phi);
  	float v = sqrt(vx*vx + vy*vy);
  	x_ << px, py, v, 0, 0;
   }
   else if(meas_package.sensor_type_==MeasurementPackage::LASER)
  {
	x_<<meas_package.raw_measurements_[0],meas_package.raw_measurements_[1],0,0,0;
	if(fabs(x_(0))<0.001 && fabs(x_(1))<0.001)
	{
	x_(0)=0.001;
	x_(1)=0.001;	
	}
  }   	
   //Intialize the weights
   weights_(0)=(1.0*lambda_)/(lambda_+n_aug_);
   for(int i=1;i<weights_.size();i++)
   {
    weights_(i)=0.5/(lambda_+n_aug_);
   }

   time_us_=meas_package.timestamp_;
   is_initialized_=true;
    return;
}
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  Prediction(dt);
  
  if(meas_package.sensor_type_==MeasurementPackage::RADAR && use_radar_)
  {
   //Call the radar
   UpdateRadar(meas_package);
  }
  if(meas_package.sensor_type_==MeasurementPackage::LASER && use_laser_)
  {
   //Call the Lidar
   UpdateLidar(meas_package); 
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
 
  // augmented mean state
  VectorXd x_aug=VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_)=x_;

  // augmented covariance matrix
  MatrixXd P_aug=MatrixXd(n_aug_,n_aug_);

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_)=P_;
  P_aug(n_x_,n_x_)=std_a_*std_a_; 
  P_aug(n_x_+1,n_x_+1)=std_yawdd_*std_yawdd_; 

  //Square root of a matrix using cholesky decomposition
  MatrixXd L=P_aug.llt().matrixL();

  //Create augmented Sigma points
  MatrixXd Xsig_aug=MatrixXd(n_aug_,n_sig_);
  Xsig_aug.col(0)=x_aug;	
  for(int i=0;i<n_aug_;i++)
  {
   Xsig_aug.col(i+1)       =x_aug+(sqrt(lambda_+n_aug_)*L.col(i));
   Xsig_aug.col(i+1+n_aug_)=x_aug-(sqrt(lambda_+n_aug_)*L.col(i));
  }

 
  //Predict sigma points
  for(int i=0;i<n_sig_;i++)
  {
   double px=Xsig_aug(0,i);
   double py=Xsig_aug(1,i);
   double v=Xsig_aug(2,i);
   double yaw=Xsig_aug(3,i);
   double yawdot=Xsig_aug(4,i);
   double nu_a=Xsig_aug(5,i);
   double nu_yawdot=Xsig_aug(6,i);

   double px_p,py_p;

   if(fabs(yawdot)>0.001)
   {
     px_p=px+v/yawdot*(sin(yaw+yawdot*delta_t)-sin(yaw));
     py_p=py+v/yawdot*(cos(yaw)-cos(yaw+yawdot*delta_t));
   }
   else
   {
     px_p=px+v*(cos(yaw))*delta_t;
     py_p=py+v*(sin(yaw))*delta_t;
   }
   double v_p=v;

   double yaw_p=yaw+yawdot*delta_t;
   double yawdot_p=yawdot;
      
   // add noise
   px_p=px_p+0.5*delta_t*delta_t*cos(yaw)*nu_a;
   py_p=py_p+0.5*delta_t*delta_t*sin(yaw)*nu_a;
      
   v_p=v_p+delta_t*nu_a;
   yaw_p=yaw_p+0.5*delta_t*delta_t*nu_yawdot;
   yawdot_p=yawdot_p+delta_t*nu_yawdot;
      
      
    Xsig_pred_(0,i)=px_p;
    Xsig_pred_(1,i)=py_p;
    Xsig_pred_(2,i)=v_p;
    Xsig_pred_(3,i)=yaw_p;
    Xsig_pred_(4,i)=yawdot_p;

  }

  //predict State mean

   x_.fill(0.0);   
   for(int i=0;i<n_sig_;i++)
   {
    x_=x_+weights_(i)*Xsig_pred_.col(i);
   }

   // Predict covariance mean;
   P_.fill(0.0);
   for(int i=0;i<n_sig_;i++)
   {
    //state difference
    VectorXd x_diff=Xsig_pred_.col(i)-x_;
    
    while(x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI; 	
    
    P_=P_+weights_(i)*x_diff*x_diff.transpose();

   }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  
  int n_z = 2;
  
  MatrixXd Zsig=Xsig_pred_.block(0,0,n_z,n_sig_);
  
  VectorXd z_pred=VectorXd(n_z);
  
  MatrixXd S=MatrixXd(n_z,n_z);

  //calc mean predicted measurement
  z_pred.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
  z_pred=z_pred+weights_(i)*Zsig.col(i);
  }

  //Calculate the Covariance matrix S
  S.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
  VectorXd z_diff=Zsig.col(i)-z_pred;
  
  while(z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  S=S+weights_(i)*z_diff*z_diff.transpose();
  
  }

  S=S+R_lidar_;

  //Matrix for cross correlation S
  MatrixXd Tc=MatrixXd(n_x_,n_z); 
  Tc.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
   //residual
   VectorXd z_diff=Zsig.col(i)-z_pred;
   // Angle normalization
   while(z_diff(1)> M_PI) z_diff(1) -=2.*M_PI;
   while(z_diff(1)<-M_PI) z_diff(1) +=2.*M_PI;
   
   VectorXd x_diff=Xsig_pred_.col(i)-x_;
   while(x_diff(3)> M_PI) x_diff(3) -=2.*M_PI;
   while(x_diff(3)<-M_PI) x_diff(3) +=2.*M_PI;

  Tc=Tc+weights_(i)*x_diff*z_diff.transpose();
  }
  
  //measurements
  VectorXd z=meas_package.raw_measurements_;

  //calc Kalman gain K
  MatrixXd K=Tc*S.inverse();

  //residuals
  VectorXd z_diff=z-z_pred;
  while(z_diff(1)> M_PI) z_diff(1) -=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1) +=2.*M_PI;
  
  //update state mean and covariance matrix
  x_=x_+K*z_diff;
  P_=P_-K*S*K.transpose();
  
  //Calc NIS values for lidar
  NIS_laser_=z_diff.transpose()*S.inverse()*z_diff;
  ofstream fout;
  fout.open("NIS_Laser_output.txt",ios::app);
  cout<<"NIS_laser="<<NIS_laser_<<endl;
  fout<<NIS_laser_;
  fout<<"\n";
  fout.close();
  //Print the state and covariance
  cout<<"x_="<<endl<<x_<<endl;
  cout<<"P_="<<endl<<P_<<endl;
	

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  MatrixXd Zsig=MatrixXd(n_z,n_sig_);
  
  VectorXd z_pred=VectorXd(n_z);
  
  MatrixXd S=MatrixXd(n_z,n_z);

  //transform sigma points to measurement steps

  for(int i=0;i<n_sig_;i++)
  {
   double p_x=Xsig_pred_(0,i);
   double p_y=Xsig_pred_(1,i);
   double v=Xsig_pred_(2,i);
   double yaw=Xsig_pred_(3,i);
   
   double v1=cos(yaw)*v;
   double v2=sin(yaw)*v;

   Zsig(0,i)=sqrt(p_x*p_x+p_y*p_y);
   Zsig(1,i)=atan2(p_y,p_x);
   Zsig(2,i)=(p_x*v1+p_y*v2)/sqrt(p_x*p_x+p_y*p_y);

  }
  //calc mean predicted measurement
  z_pred.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
  z_pred=z_pred+weights_(i)*Zsig.col(i);
  }

  //Calculate the Covariance matrix S
  S.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
  VectorXd z_diff=Zsig.col(i)-z_pred;
  
  while(z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  S=S+weights_(i)*z_diff*z_diff.transpose();
  
  }

  S=S+R_radar_;

  //Matrix for cross correlation S
  MatrixXd Tc=MatrixXd(n_x_,n_z); 
  Tc.fill(0.0);
  for(int i=0;i<n_sig_;i++)
  {
   //residual
   VectorXd z_diff=Zsig.col(i)-z_pred;
   // Angle normalization
   while(z_diff(1)> M_PI) z_diff(1) -=2.*M_PI;
   while(z_diff(1)<-M_PI) z_diff(1) +=2.*M_PI;
   
   VectorXd x_diff=Xsig_pred_.col(i)-x_;
   while(x_diff(3)> M_PI) x_diff(3) -=2.*M_PI;
   while(x_diff(3)<-M_PI) x_diff(3) +=2.*M_PI;

  Tc=Tc+weights_(i)*x_diff*z_diff.transpose();
  }
  
  //measurements
  VectorXd z=meas_package.raw_measurements_;

  //calc Kalman gain K
  MatrixXd K=Tc*S.inverse();

  //residuals
  VectorXd z_diff=z-z_pred;
  while(z_diff(1)> M_PI) z_diff(1) -=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1) +=2.*M_PI;
  
  //update state mean and covariance matrix
  x_=x_+K*z_diff;
  P_=P_-K*S*K.transpose();
  
  //Calc the NIS values for radar
  NIS_radar_=z_diff.transpose()*S.inverse()*z_diff;
  ofstream fout;
  fout.open("NIS_radar_output.txt",ios::app);
  cout<<"NIS_radar="<<NIS_radar_<<endl;
  fout<<NIS_radar_;
  fout<<"\n";
  fout.close();

  //Print the state and covariance	
  cout<<"x_="<<endl<<x_<<endl;
  cout<<"P_="<<endl<<P_<<endl;
}
