#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        std::cout << "Incompatible size for rmse" << std::endl;
        return rmse;
    }
    
    for (int i = 0; i < estimations.size(); i++) {
        VectorXd diff = estimations[i] - ground_truth[i];
        VectorXd sq = diff.array() * diff.array();
        
        rmse += sq;
    }
    
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];
    
    float qs1 = px * px + py * py;
    float qs2 = sqrt(qs1);
    float qs3 = qs1 * qs2;
    
    float pyq = py / qs2;
    float pxq = px / qs2;
    
    MatrixXd hj = MatrixXd(3,4);
    
    hj << pxq, pyq, 0, 0,
    -px/qs1, px/qs1, 0, 0,
    py*(vx*py-vy*px)/qs3, px*(vy*px-vx*py)/qs3,pxq,pyq;
    
    return hj;
}
