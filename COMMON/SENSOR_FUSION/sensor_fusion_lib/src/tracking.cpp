//
// Created by apple on 05/04/2017.
//

#include "tracking.h"
//#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tracking::Tracking() {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    //create a 4D state vector, we don't know yet the values of the x state
    kf_.x_ = VectorXd(4);

    //state covariance matrix P
    kf_.P_ = MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


    //measurement covariance
    kf_.R_ = MatrixXd(2, 2);
    kf_.R_ << 0.0225, 0,
            0, 0.0225;

    //measurement matrix
    kf_.H_ = MatrixXd(2, 4);
    kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //the initial transition matrix F_
    kf_.F_ = MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //set the acceleration noise components
    noise_ax = 5;
    noise_ay = 5;

}



Tracking::Tracking(MatrixXd R, float ax_noise, float ay_noise) {
    is_initialized_ = false;
    previous_timestamp_ = 0;

    //create a 4D state vector, we don't know yet the values of the x state
    kf_.x_ = VectorXd(4);

    //state covariance matrix P
    kf_.P_ = MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


    //measurement covariance
    kf_.R_ = MatrixXd(2, 2);
    kf_.R_ = R;

    //measurement matrix
    kf_.H_ = MatrixXd(2, 4);
    kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //the initial transition matrix F_
    kf_.F_ = MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //set the acceleration noise components
    noise_ax = ax_noise;
    noise_ay = ay_noise;

}



Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        //cout << "Kalman Filter Initialization " << endl;

        //set the state with the initial location and zero velocity
        kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.1, 0.1;

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_);	//dt - expressed in seconds
    std::cout << "dt = " << dt << std::endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    //1. Modify the F matrix so that the time is integrated
    //2. Set the process covariance matrix Q
    //3. Call the Kalman Filter predict() function
    //4. Call the Kalman Filter update() function
    // with the most recent raw measurements_
    kf_.F_<< 1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;

    double dt_2 = pow(dt,2);
    double dt_3 = pow(dt,3);
    double dt_4 = pow(dt,4);

    kf_.Q_ = MatrixXd(4, 4);
    kf_.Q_<< dt_4*noise_ax/4, 0, dt_3*noise_ax/2, 0,
            0, dt_4*noise_ay/4, 0, dt_3*noise_ay/2,
            dt_3*noise_ax/2, 0, dt_2*noise_ax, 0,
            0, dt_3*noise_ay/2, 0, dt_2*noise_ay;

    kf_.Predict();
    kf_.Update(measurement_pack.raw_measurements_);

//    std::cout << "x_= " << kf_.x_ << std::endl;
//    std::cout << "noise_ax= " << noise_ax << std::endl;
//    std::cout << "R = " << kf_.Q_ << std::endl;
//    std::cout << "P_= " << kf_.P_ << std::endl;

}
