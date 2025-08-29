#include "tracking.h"

KalmanFilter2D::KalmanFilter2D(float dt, float process_noise, float measurement_noise) {
    // Stato: [x, y, vx, vy]^T
    x_ = Eigen::Vector4f::Zero();

    // Matrice di transizione (modello costante velocità)
    F_ = Eigen::Matrix4f::Identity();
    F_(0,2) = dt;
    F_(1,3) = dt;

    // Matrice di osservazione (misuriamo solo posizione x,y)
    H_ = Eigen::Matrix<float, 2, 4>::Zero();
    H_(0,0) = 1;
    H_(1,1) = 1;

    // Covarianza iniziale
    P_ = Eigen::Matrix4f::Identity() * 1000;

    // Rumore di processo
    Q_ = Eigen::Matrix4f::Identity() * process_noise;

    // Rumore di misura
    R_ = Eigen::Matrix2f::Identity() * measurement_noise;
}

void KalmanFilter2D::init(float x, float y) {
    x_ << x, y, 0, 0;  // posizione iniziale + velocità = 0
}

void KalmanFilter2D::predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter2D::update(float meas_x, float meas_y) {
    Eigen::Vector2f z(meas_x, meas_y);
    Eigen::Vector2f y = z - H_ * x_;   // residual
    Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<float,4,2> K = P_ * H_.transpose() * S.inverse(); // gain
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix4f::Identity() - K * H_) * P_;
}

