//
// Created by lihao on 2022/10/3.
//

#ifndef SENSORFUSION_GROUND_TRUTH_PACKAGE_H
#define SENSORFUSION_GROUND_TRUTH_PACKAGE_H

#include <iostream>
#include <iomanip>
#include "Eigen/Dense"


class GroundTruthPackage {
public:
    long long timestamp_{};

    Eigen::VectorXd gt_values_;


    friend std::ostream& operator<<(std::ostream& cout, GroundTruthPackage& ground_truth)
    {
        cout << ground_truth.timestamp_ << " " << std::scientific << std::setprecision(6) << std::setw(15) <<
        ground_truth.gt_values_.transpose();
        return cout;
    }

};


#endif //SENSORFUSION_GROUND_TRUTH_PACKAGE_H
