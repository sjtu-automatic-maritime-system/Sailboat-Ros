//
// Created by apple on 05/04/2017.
//

#ifndef SENSORFUSION_KF_TRACKING_H
#define SENSORFUSION_KF_TRACKING_H

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kf_2d.h"


class Tracking {
public:
    void InitTracking(MatrixXd _R, float _noise_ax, float _noise_ay, float _init_vx=0, float _init_vy=0);

//    Tracking(MatrixXd R, float ax_noise, float ay_noise);

    virtual ~Tracking();

    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    KalmanFilter kf_;

private:
    bool is_initialized_;
    long previous_timestamp_;

    //acceleration noise components
    float noise_ax;
    float noise_ay;
    float init_vx;
    float init_vy;

};

#endif //SENSORFUSION_KF_TRACKING_H
