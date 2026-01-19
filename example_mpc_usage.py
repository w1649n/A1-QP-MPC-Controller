#!/usr/bin/env python3
"""
Example: Using MPC Controller from Python

This example demonstrates basic usage of the MPC controller Python bindings.
"""

import numpy as np
import mpc_controller


def main():
    print("=" * 60)
    print("A1 MPC Controller Python Binding Example")
    print("=" * 60)
    
    # Step 1: Create MPC controller
    print("\n1. Creating MPC controller...")
    mpc = mpc_controller.ConvexMpc()
    print("   ✓ MPC controller created")
    
    # Step 2: Set MPC weights
    print("\n2. Setting MPC weights...")
    # These are the default weights from A1CtrlStates
    q_weights = np.array([
        80.0, 80.0, 1.0,      # roll, pitch, yaw weights
        0.0, 0.0, 270.0,      # x, y, z position weights
        1.0, 1.0, 20.0,       # angular velocity weights
        20.0, 20.0, 20.0,     # linear velocity weights
        0.0                   # gravity placeholder
    ])
    
    r_weights = np.array([
        1e-5, 1e-5, 1e-6,     # FL leg forces
        1e-5, 1e-5, 1e-6,     # FR leg forces
        1e-5, 1e-5, 1e-6,     # RL leg forces
        1e-5, 1e-5, 1e-6      # RR leg forces
    ])
    
    mpc.set_weights(q_weights, r_weights)
    print("   ✓ Weights configured")
    
    # Step 3: Create robot state
    print("\n3. Creating robot state...")
    state = mpc_controller.MPCState()
    
    # Current state
    state.root_euler = np.array([0.0, 0.0, 0.0])  # No rotation
    state.root_pos = np.array([0.0, 0.0, 0.3])    # 30cm height
    state.root_ang_vel = np.array([0.0, 0.0, 0.0])
    state.root_lin_vel = np.array([0.0, 0.0, 0.0])
    
    # Foot positions in absolute frame (centered at robot origin)
    # FL, FR, RL, RR
    state.foot_pos = np.array([
        [ 0.17,  0.17, -0.17, -0.17],  # x coordinates
        [ 0.15, -0.15,  0.15, -0.15],  # y coordinates
        [-0.05, -0.05, -0.05, -0.05]   # z coordinates (5cm below body)
    ])
    
    # All feet in contact
    state.contacts = [True, True, True, True]
    
    # Desired state (want to move forward at 0.5 m/s)
    state.root_pos_d = np.array([0.0, 0.0, 0.3])
    state.root_euler_d = np.array([0.0, 0.0, 0.0])
    state.root_lin_vel_d = np.array([0.5, 0.0, 0.0])  # Forward velocity
    state.root_ang_vel_d = np.array([0.0, 0.0, 0.0])
    
    print("   ✓ State configured")
    print(f"      Current position: {state.root_pos}")
    print(f"      Desired velocity: {state.root_lin_vel_d}")
    
    # Step 4: Solve MPC
    print("\n4. Solving MPC...")
    grf = mpc.solve(state, dt=0.0025)
    print("   ✓ MPC solved successfully")
    
    # Step 5: Display results
    print("\n5. Results:")
    print("   Ground Reaction Forces (in body frame):")
    print(f"      FL: [{grf[0]:7.2f}, {grf[1]:7.2f}, {grf[2]:7.2f}] N")
    print(f"      FR: [{grf[3]:7.2f}, {grf[4]:7.2f}, {grf[5]:7.2f}] N")
    print(f"      RL: [{grf[6]:7.2f}, {grf[7]:7.2f}, {grf[8]:7.2f}] N")
    print(f"      RR: [{grf[9]:7.2f}, {grf[10]:7.2f}, {grf[11]:7.2f}] N")
    
    total_vertical = grf[2] + grf[5] + grf[8] + grf[11]
    print(f"\n   Total vertical force: {total_vertical:.2f} N")
    print(f"   Expected (robot weight): {state.robot_mass * 9.8:.2f} N")
    
    # Step 6: Test with RL-style weights
    print("\n6. Testing with RL-MPC weights...")
    rl_q_weights = np.array([
        1.0, 1.5, 0.0,        # Lighter orientation weights
        0.0, 0.0, 50.0,       # Focus on height control
        0.0, 0.0, 0.1,        # Less angular velocity penalty
        1.0, 1.0, 0.1,        # Moderate velocity tracking
        0.0
    ])
    
    mpc.set_weights(rl_q_weights, r_weights)
    grf_rl = mpc.solve(state, dt=0.0025)
    print("   ✓ MPC solved with RL weights")
    print(f"   Total vertical force: {grf_rl[2] + grf_rl[5] + grf_rl[8] + grf_rl[11]:.2f} N")
    
    print("\n" + "=" * 60)
    print("Example completed successfully!")
    print("=" * 60)
    print("\nNext steps:")
    print("  - Integrate with your RL policy to output q_weights")
    print("  - Use in Isaac Lab simulation")
    print("  - Tune weights for your specific task")
    

if __name__ == "__main__":
    main()
