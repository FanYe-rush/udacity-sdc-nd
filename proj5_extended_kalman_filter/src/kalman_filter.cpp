#include "kalman_filter.h"
#include <iostream>
#include <math.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_lidar_in, MatrixXd &R_radar_in,
                        float ax_noise_in, float ay_noise_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_lidar_ = R_lidar_in;
    R_radar_ = R_radar_in;
    noise_ax = ax_noise_in;
    noise_ay = ay_noise_in;
}

void KalmanFilter::Predict(float delta_T) {
    float dt2 = delta_T * delta_T;
    float dt3 = dt2 * delta_T;
    float dt4 = dt3 * delta_T;
    
    dt4 /= 4;
    dt3 /= 2;
    
    MatrixXd Q = MatrixXd(4,4);
    Q << dt4*noise_ax,0,dt3*noise_ax,0,
         0,dt4*noise_ay,0,dt3*noise_ay,
         dt3*noise_ax,0,dt2*noise_ax,0,
         0,dt3*noise_ay,0,dt2*noise_ay;
    
    
    F_(0,2) = delta_T;
    F_(1,3) = delta_T;
    
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z, const bool isRadarData) {
    MatrixXd H = H_;
    MatrixXd R = R_lidar_;
    
    VectorXd y;
    
    if (isRadarData) {
        H = tools.CalculateJacobian(x_);
        R = R_radar_;
        y = VectorXd(3);
        float dist = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
        y << dist, atan2(x_(1), x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/dist;
        y = z - y;
    } else {
        y = z - H * x_;
    }
    
    MatrixXd S = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    x_ = x_ + K * y;
    P_ = (I - K * H) * P_;
}
