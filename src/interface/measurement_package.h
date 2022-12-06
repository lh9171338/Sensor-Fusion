//
// Created by lihao on 2022/10/3.
//

#ifndef SENSORFUSION_MEASUREMENT_PACKAGE_H
#define SENSORFUSION_MEASUREMENT_PACKAGE_H

#include <iostream>
#include <iomanip>
#include "Eigen/Dense"


class MeasurementPackage {
public:
    long long timestamp_{};

    enum SensorType{
        Unknown,
        LIDAR,
        RADAR,
        FUSION,
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    MeasurementPackage() : sensor_type_(Unknown) {}

    friend std::ostream& operator<<(std::ostream& cout, MeasurementPackage& measurement)
    {
        if(measurement.sensor_type_ == LIDAR) {
            cout << "L ";
        } else if(measurement.sensor_type_ == RADAR) {
            cout << "R ";
        } else if(measurement.sensor_type_ == FUSION) {
            cout << "F ";
        }
        cout << measurement.timestamp_ << " " << std::scientific << std::setprecision(6) << std::setw(15) <<
        measurement.raw_measurements_.transpose();
        return cout;
    }

};


#endif //SENSORFUSION_MEASUREMENT_PACKAGE_H
