//
//  TestKalmanFilter.cpp
//  ExtendedKF
//
//  Created by Bajrang Chapola on 17/08/18.
//

#include "TestKalmanFilter.hpp"
#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter Variables
VectorXd x; //Object state
MatrixXd P; //Object Covariance matrix
MatrixXd u; //External motion
MatrixXd F; //State transition matrix
MatrixXd H; // Measurement matrix
MatrixXd R; // MEasurement Covariance matrix
MatrixXd I; //Identity matrix
MatrixXd Q; //process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x,MatrixXd &P);
int main(){
    
    x=VectorXd(2);
    x<<0,0;
    
    P = MatrixXd(2,2);
    P<<1000,0,0,1000;
    
    u = VectorXd(2);
    u << 0, 0;
    
    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;
    
    H = MatrixXd(1, 2);
    H << 1, 0;
    
    R = MatrixXd(1, 1);
    R << 1;
    
    I = MatrixXd::Identity(2, 2);
    
    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;
    
    //Create a list of measurement
    VectorXd single_meas(1);
    single_meas<<1;
    
   
    measurements.push_back(single_meas);
    single_meas << 2;
    measurements.push_back(single_meas);
    single_meas << 3;
    measurements.push_back(single_meas);
    
    //call Kalman filter alogorithm
    
    filter(x, P);
    
    
    
    return 0;
}
void filter(VectorXd &x,MatrixXd &P){
    for (unsigned int n=0; n<measurements.size(); ++n) {
        VectorXd z = measurements[n];
        VectorXd y = z - H *x;
        MatrixXd Ht  = H.transpose();
        MatrixXd S = H*P*Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd K = P*Ht*Si;
        
        
        // new state
        x= x+(K*y);
        P = (I-K*H)+P;
        
        // KF Prediction step
        
        x = F*x+u;
        MatrixXd Ft = F.transpose();
        P = F*P*Ft+Q;
        
        
        std::cout << "x=" << std::endl <<  x << std::endl;
        std::cout << "P=" << std::endl <<  P << std::endl;
        

    }
}

