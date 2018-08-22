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
    VectorXd rsme(4);
    rsme<<0,0,0,0;
    
    //Cehck Validity
    
    if (estimations.size()!=ground_truth.size() || estimations.size()==0) {
        cout<<"Invalid estimation or ground_truth data" << endl;
        return rsme;
    }
    for (unsigned int i=0; i<estimations.size(); ++i) {
        VectorXd residual = estimations[i]-ground_truth[i];
        
        //coefficient-wise multiplication
        residual =residual.array()*residual.array();
        rsme+=residual;
    }
    
    // Calculate the mean
    
    rsme = rsme/estimations.size();
    
    //Calculate the Squre Root
    
    rsme = rsme.array().sqrt();
    
    
    return rsme;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    /**
     TODO:
     * Calculate a Jacobian here.
     */
    MatrixXd Hj=MatrixXd(3,4);
    
    //Reciver State Parameter
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = c1*c2;
    
    // Check Division by 0
    if (abs(c1)<0.0001) {
        cout<<"Calculate Jacobian Error - Division by zero";
        return Hj;
    }
    
    // compute Jacobian Matrix
    Hj<<(px/c2),(py/c2),0,0,
    -(py/c1),(px/c1),0,0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    
    return Hj;
}
