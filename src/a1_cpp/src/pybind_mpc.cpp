//
// Python bindings for MPC controller
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "ConvexMpc.h"
#include "MPCState.h"
#include "A1Params.h"
#include "OsqpEigen/OsqpEigen.h"
#include "utils/Utils.h"

namespace py = pybind11;

// Wrapper class for ConvexMpc to provide a Python-friendly interface
class ConvexMpcPython {
public:
    ConvexMpcPython() {
        // Initialize with default weights
        q_weights.resize(13);
        r_weights.resize(12);
        
        // Default weights from A1CtrlStates
        q_weights << 80.0, 80.0, 1.0,
                     0.0, 0.0, 270.0,
                     1.0, 1.0, 20.0,
                     20.0, 20.0, 20.0,
                     0.0;
        r_weights << 1e-5, 1e-5, 1e-6,
                     1e-5, 1e-5, 1e-6,
                     1e-5, 1e-5, 1e-6,
                     1e-5, 1e-5, 1e-6;
        
        mpc_solver = std::make_shared<ConvexMpc>(q_weights, r_weights);
        solver_initialized = false;
    }
    
    void set_weights(const Eigen::VectorXd &q_weights_new, const Eigen::VectorXd &r_weights_new) {
        if (q_weights_new.size() != 13) {
            throw std::runtime_error("q_weights must be 13-dimensional");
        }
        if (r_weights_new.size() != 12) {
            throw std::runtime_error("r_weights must be 12-dimensional");
        }
        
        q_weights = q_weights_new;
        r_weights = r_weights_new;
        mpc_solver->set_weights(q_weights, r_weights);
    }
    
    void reset() {
        mpc_solver->reset();
        solver_initialized = false;
    }
    
    Eigen::VectorXd solve(MPCState &state, double dt = 0.0025, bool reset_solver = false) {
        // Optionally reset solver state
        if (reset_solver) {
            mpc_solver->reset();
        }
        
        // Build MPC state vector
        Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
        mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                      state.root_pos[0], state.root_pos[1], state.root_pos[2],
                      state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                      state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
                      -9.8;
        
        // Compute rotation matrix from euler angles
        // ZYX convention (yaw-pitch-roll)
        Eigen::Matrix3d root_rot_mat = 
            Eigen::AngleAxisd(state.root_euler[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(state.root_euler[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(state.root_euler[0], Eigen::Vector3d::UnitX());
        Eigen::Vector3d root_lin_vel_d_world = root_rot_mat * state.root_lin_vel_d;
        
        // Build desired trajectory
        Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;
        for (int i = 0; i < PLAN_HORIZON; ++i) {
            mpc_states_d.segment(i * MPC_STATE_DIM, MPC_STATE_DIM)
                    << state.root_euler_d[0],
                       state.root_euler_d[1],
                       state.root_euler[2] + state.root_ang_vel_d[2] * dt * (i + 1),
                       state.root_pos[0] + root_lin_vel_d_world[0] * dt * (i + 1),
                       state.root_pos[1] + root_lin_vel_d_world[1] * dt * (i + 1),
                       state.root_pos_d[2],
                       state.root_ang_vel_d[0],
                       state.root_ang_vel_d[1],
                       state.root_ang_vel_d[2],
                       root_lin_vel_d_world[0],
                       root_lin_vel_d_world[1],
                       0,
                       -9.8;
        }
        
        // Calculate A matrix (single for entire trajectory)
        mpc_solver->calculate_A_mat_c(state.root_euler);
        
        // Calculate B matrices for each horizon point
        for (int i = 0; i < PLAN_HORIZON; i++) {
            mpc_solver->calculate_B_mat_c(state.robot_mass,
                                          state.trunk_inertia,
                                          root_rot_mat,
                                          state.foot_pos);
            
            // State space discretization
            mpc_solver->state_space_discretization(dt);
            
            // Store current B_d matrix
            mpc_solver->B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(i * MPC_STATE_DIM, 0) = mpc_solver->B_mat_d;
        }
        
        // Calculate QP matrices using the overloaded version
        mpc_solver->calculate_qp_mats(mpc_states, mpc_states_d, state.contacts);
        
        // Solve QP
        if (!solver_initialized) {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver->linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver->hessian);
            solver.data()->setGradient(mpc_solver->gradient);
            solver.data()->setLowerBound(mpc_solver->lb);
            solver.data()->setUpperBound(mpc_solver->ub);
            solver.initSolver();
            solver_initialized = true;
        } else {
            solver.updateHessianMatrix(mpc_solver->hessian);
            solver.updateGradient(mpc_solver->gradient);
            solver.updateLowerBound(mpc_solver->lb);
            solver.updateUpperBound(mpc_solver->ub);
        }
        
        solver.solve();
        
        // Get solution
        Eigen::VectorXd solution = solver.getSolution();
        
        // Convert to body frame forces
        Eigen::VectorXd grf(12);
        for (int i = 0; i < NUM_LEG; ++i) {
            grf.segment<3>(i * 3) = root_rot_mat.transpose() * solution.segment<3>(i * 3);
        }
        
        return grf;
    }
    
private:
    std::shared_ptr<ConvexMpc> mpc_solver;
    OsqpEigen::Solver solver;
    bool solver_initialized;
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;
};

PYBIND11_MODULE(mpc_controller, m) {
    m.doc() = "Python bindings for A1 MPC Controller";
    
    // Bind MPCState structure
    py::class_<MPCState>(m, "MPCState")
        .def(py::init<>())
        .def_readwrite("root_euler", &MPCState::root_euler)
        .def_readwrite("root_pos", &MPCState::root_pos)
        .def_readwrite("root_ang_vel", &MPCState::root_ang_vel)
        .def_readwrite("root_lin_vel", &MPCState::root_lin_vel)
        .def_readwrite("foot_pos", &MPCState::foot_pos)
        .def_property("contacts",
            [](const MPCState &s) {
                std::vector<bool> contacts_vec(NUM_LEG);
                for (int i = 0; i < NUM_LEG; ++i) {
                    contacts_vec[i] = s.contacts[i];
                }
                return contacts_vec;
            },
            [](MPCState &s, const std::vector<bool> &contacts_vec) {
                if (contacts_vec.size() != NUM_LEG) {
                    throw std::runtime_error("contacts must have 4 elements");
                }
                for (int i = 0; i < NUM_LEG; ++i) {
                    s.contacts[i] = contacts_vec[i];
                }
            })
        .def_readwrite("robot_mass", &MPCState::robot_mass)
        .def_readwrite("trunk_inertia", &MPCState::trunk_inertia)
        .def_readwrite("root_euler_d", &MPCState::root_euler_d)
        .def_readwrite("root_pos_d", &MPCState::root_pos_d)
        .def_readwrite("root_ang_vel_d", &MPCState::root_ang_vel_d)
        .def_readwrite("root_lin_vel_d", &MPCState::root_lin_vel_d);
    
    // Bind ConvexMpcPython class
    py::class_<ConvexMpcPython>(m, "ConvexMpc")
        .def(py::init<>())
        .def("set_weights", &ConvexMpcPython::set_weights,
             py::arg("q_weights"), py::arg("r_weights"),
             "Set MPC weights (q_weights: 13-dim, r_weights: 12-dim)")
        .def("solve", &ConvexMpcPython::solve,
             py::arg("state"), py::arg("dt") = 0.0025, py::arg("reset_solver") = false,
             "Solve MPC and return ground reaction forces (12-dim vector)")
        .def("reset", &ConvexMpcPython::reset,
             "Reset the MPC solver");
}
