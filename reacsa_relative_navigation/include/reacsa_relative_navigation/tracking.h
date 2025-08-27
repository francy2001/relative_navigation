#ifndef TRACKING_H
#define TRACKING_H

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter2D {
public:
    KalmanFilter2D(float dt, float process_noise, float measurement_noise);

    void init(float x, float y);

    void predict();

    void update(float meas_x, float meas_y);

    float getX() const { return x_(0); }
    float getY() const { return x_(1); }

private:
    Eigen::Vector4f x_;   // stato [x,y,vx,vy]
    Eigen::Matrix4f F_;   // transizione
    Eigen::Matrix<float,2,4> H_; // osservazione
    Eigen::Matrix4f P_;   // covarianza
    Eigen::Matrix4f Q_;   // rumore di processo
    Eigen::Matrix2f R_;   // rumore di misura
};


#endif // TRACKING_H