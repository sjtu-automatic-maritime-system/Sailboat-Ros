//
// Created by apple on 05/04/2017.
//

#ifndef SENSORFUSION_KF_MEASUREMENT_PACKAGE_H
#define SENSORFUSION_KF_MEASUREMENT_PACKAGE_H

#include <eigen3/Eigen/Eigen>

class MeasurementPackage {
public:
    float timestamp_;

    enum SensorType {
        LASER, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

};

#endif //SENSORFUSION_KF_MEASUREMENT_PACKAGE_H
