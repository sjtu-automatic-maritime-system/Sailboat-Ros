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
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    long previous_timestamp_;

    //acceleration noise components
    float noise_ax;
    float noise_ay;

};

#endif //SENSORFUSION_KF_TRACKING_H
