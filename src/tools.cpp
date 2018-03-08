#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse<<0,0,0,0;
	
	//Check the size of the estimation array
	if(estimations.size()!=ground_truth.size()||estimations.size()==0)
	{
		std::cout<<"Invalid Estimation size"<<endl;
		return rmse;
	}
	
	//loop through the estimation array and calc the residuals
	for(unsigned int i=0;i<estimations.size();++i)
	{
		VectorXd residual=estimations[i]-ground_truth[i];
		residual=residual.array()*residual.array();
		rmse+=residual;
	}
	
	//calc the mean of residuals
	rmse=rmse/estimations.size();
	
	//sqrt of the residuals
	rmse=rmse.array().sqrt();
	
	return rmse;
}
