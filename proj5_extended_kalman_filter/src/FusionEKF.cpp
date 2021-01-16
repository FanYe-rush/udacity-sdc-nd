#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    
    // measurement matrix for laser data
    H_laser_ << 1,0,0,0,
                0,1,0,0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;
   
        MatrixXd F = MatrixXd(4,4);
        F << 1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1;
        
        MatrixXd P = MatrixXd(4,4);
        P << 1,0,0,0,
             0,1,0,0,
             0,0,1000,0,
             0,0,0,1000;
        
        float noise_ax = 9;
        float noise_ay = 9;
        
        VectorXd first_measure = VectorXd(4);
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            MatrixXd hj = tools.CalculateJacobian(measurement_pack.raw_measurements_);
            first_measure = hj.inverse() * measurement_pack.raw_measurements_;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            VectorXd current_p = measurement_pack.raw_measurements_;
            first_measure << current_p(0), current_p(1), 0, 0;
        }
        
        ekf_.Init(first_measure, P, F, H_laser_, R_laser_, R_radar_, noise_ax, noise_ay);
        previous_timestamp_ = measurement_pack.timestamp_;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        
        return;
    }
    
    /**
     * Prediction
     */
    
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    ekf_.Predict(dt);
    
    /**
     * Update
     */
    
    bool isRadarData = (measurement_pack.sensor_type_ == MeasurementPackage::RADAR);
    ekf_.Update(measurement_pack.raw_measurements_, isRadarData);
    
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
}
