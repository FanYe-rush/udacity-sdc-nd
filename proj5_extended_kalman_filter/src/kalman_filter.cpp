#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_lidar_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_lidar_ = R_lidar_in;
    R_radar_ = R_radar_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict(float delta_T) {
    F_[0,2] = delta_T;
    F_[1,3] = delta_T;
    
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, const bool isRadarData) {
    MatrixXd H = H_;
    MatrixXd R = R_lidar_;
    
    if (isRadarData) {
        H = tools.CalculateJacobian(x_);
        R = R_radar_;
    }
    
    VectorXd y = z - H * x_;
    MatrixXd S = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    x_ = x_ + K * y;
    P_ = (I - K * H) * P_;
}
