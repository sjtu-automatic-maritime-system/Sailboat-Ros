//
// Created by apple on 05/04/2017.
//

#ifndef SENSORFUSION_KF_KF_2D_H
#define SENSORFUSION_KF_KF_2D_H

#include <eigen3/Eigen/Eigen>

using Eigen::MatrixXd;
using Eigen::VectorXd;


class KalmanFilter {

public:

    ///* state vector
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* state transistion matrix
    MatrixXd F_;

    ///* process covariance matrix
    MatrixXd Q_;

    ///* measurement matrix
    MatrixXd H_;

    ///* measurement covariance matrix
    MatrixXd R_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Predict Predicts the state and the state covariance
     * using the process model
     */
    void Predict();

    /**
     * Updates the state and
     * @param z The measurement at k+1
     */
    void Update(const VectorXd &z);

};


#endif //SENSORFUSION_KF_KF_2D_H
