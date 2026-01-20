//
// Created for Python binding
//

#ifndef A1_CPP_MPCSTATE_H
#define A1_CPP_MPCSTATE_H

#include <Eigen/Dense>
#include "A1Params.h"

// Simplified state structure for Python binding (no ROS dependencies)
struct MPCState {
    // Robot state (13 dimensions)
    Eigen::Vector3d root_euler;      // roll, pitch, yaw
    Eigen::Vector3d root_pos;        // x, y, z position
    Eigen::Vector3d root_ang_vel;    // angular velocity in body frame
    Eigen::Vector3d root_lin_vel;    // linear velocity in body frame
    
    // Foot positions in absolute frame (3x4 matrix)
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos;
    
    // Contact states (4 bool values)
    bool contacts[NUM_LEG];
    
    // Robot properties
    double robot_mass;
    Eigen::Matrix3d trunk_inertia;
    
    // Desired trajectory (for MPC horizon)
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_lin_vel_d;
    
    MPCState() {
        root_euler.setZero();
        root_pos.setZero();
        root_ang_vel.setZero();
        root_lin_vel.setZero();
        foot_pos.setZero();
        for (int i = 0; i < NUM_LEG; ++i) {
            contacts[i] = false;
        }
        robot_mass = 15.0;  // Default A1 mass
        trunk_inertia << 0.0158533, 0.0, 0.0,
                         0.0, 0.0377999, 0.0,
                         0.0, 0.0, 0.0456542;
        root_euler_d.setZero();
        root_pos_d.setZero();
        root_ang_vel_d.setZero();
        root_lin_vel_d.setZero();
    }
};

#endif //A1_CPP_MPCSTATE_H
