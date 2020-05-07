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
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /**
     * TODO: Finish initializing the FusionEKF.
     * TODO: Set the process and measurement noises
     */

    // Initialize P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

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
        /**
         * TODO: Initialize the state ekf_.x_ with the first measurement.
         * TODO: Create the covariance matrix.
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // TODO: Convert radar from polar to cartesian coordinates
            //         and initialize state.
            double rho = measurement_pack.raw_measurements_[0]; // range
            double theta = measurement_pack.raw_measurements_[1]; // bearing
            double rho_theta = measurement_pack.raw_measurements_[2]; // velocity of rho
            // Convertion from polar to cartesian Coordinates
            cout << "rho: " << rho << endl;
            cout << "phi: " << theta << endl;
            cout << "rho_theta: " << rho_theta << endl;

            ekf_.x_(0) = rho * cos(theta);
            ekf_.x_(1) = rho * sin(theta);
            ekf_.x_(2) = rho_theta * cos(theta);
            ekf_.x_(3) = rho_theta * sin(theta);

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // TODO: Initialize state.
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
            ekf_.x_(2)=0;
            ekf_.x_(3)=0;

        }

        // done initializing, no need to predict or update
        cout << "EKF init: " << ekf_.x_ << endl;
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    /**
     * TODO: Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * TODO: Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */

    float dt = (measurement_pack.timestamp_ -previous_timestamp_)/1000000.0;
    previous_timestamp_= measurement_pack.timestamp_;
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    float noise_ax= 9.0;
    float noise_ay =9.0;

    double dt2 = dt*dt;
    double dt3 = dt*dt*dt;
    double dt4 = dt*dt*dt*dt;

    ekf_.Q_= MatrixXd(4,4);
    ekf_.Q_<<dt4/4*noise_ax,0,dt3/2* noise_ax,0,
            0,dt4/4* noise_ay,0,dt3/2*noise_ay,
            dt3/2*noise_ax,0,dt2*noise_ax,0,
            0,dt3/2*noise_ay,0,dt2*noise_ay;

    ekf_.Predict();

    /**
     * Update
     */

    /**
     * TODO:
     * - Use the sensor type to perform the update step.
     * - Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // TODO: Radar updates
        Hj_=tools.CalculateJacobian(ekf_.x_);
        ekf_.H_=Hj_;
        ekf_.R_=R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        // TODO: Laser updates
        ekf_.H_=H_laser_;
        ekf_.R_= R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);

    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
