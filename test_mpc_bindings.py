#!/usr/bin/env python3
"""
Test script for MPC Python bindings

This script demonstrates how to use the MPC controller from Python.
"""

import numpy as np

try:
    import mpc_controller
    print("✓ Successfully imported mpc_controller module")
except ImportError as e:
    print(f"✗ Failed to import mpc_controller: {e}")
    print("\nPlease build and install the module first:")
    print("  pip install -e .")
    exit(1)


def test_mpc_state():
    """Test MPCState creation and attribute access"""
    print("\n=== Testing MPCState ===")
    
    state = mpc_controller.MPCState()
    print("✓ Created MPCState instance")
    
    # Test setting state values
    state.root_euler = np.array([0.0, 0.0, 0.0])
    state.root_pos = np.array([0.0, 0.0, 0.3])
    state.root_ang_vel = np.array([0.0, 0.0, 0.0])
    state.root_lin_vel = np.array([0.0, 0.0, 0.0])
    print("✓ Set robot state values")
    
    # Test foot positions (3x4 matrix)
    state.foot_pos = np.array([
        [0.17, 0.17, -0.17, -0.17],   # x positions
        [0.15, -0.15, 0.15, -0.15],   # y positions
        [-0.05, -0.05, -0.05, -0.05]  # z positions
    ])
    print("✓ Set foot positions")
    
    # Test contacts
    state.contacts = [True, True, True, True]
    print("✓ Set contact states")
    
    # Test desired values
    state.root_pos_d = np.array([0.0, 0.0, 0.3])
    state.root_euler_d = np.array([0.0, 0.0, 0.0])
    state.root_lin_vel_d = np.array([0.5, 0.0, 0.0])  # Move forward
    state.root_ang_vel_d = np.array([0.0, 0.0, 0.0])
    print("✓ Set desired trajectory")
    
    return state


def test_mpc_controller(state):
    """Test ConvexMpc controller"""
    print("\n=== Testing ConvexMpc Controller ===")
    
    # Create MPC controller
    mpc = mpc_controller.ConvexMpc()
    print("✓ Created ConvexMpc instance")
    
    # Test set_weights
    q_weights = np.array([80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0])
    r_weights = np.array([1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6])
    mpc.set_weights(q_weights, r_weights)
    print("✓ Set MPC weights")
    
    # Test solve
    try:
        grf = mpc.solve(state, dt=0.0025)
        print(f"✓ MPC solve successful, GRF shape: {grf.shape}")
        print(f"  Ground Reaction Forces (12-dim):\n  {grf}")
        
        # Check if the output makes sense
        if grf.shape == (12,):
            print("✓ Output shape is correct (12,)")
        else:
            print(f"✗ Output shape is incorrect: {grf.shape}")
            
        # The forces should be reasonable (not NaN or too large)
        if np.all(np.isfinite(grf)) and np.all(np.abs(grf) < 1000):
            print("✓ Output values are reasonable")
        else:
            print("✗ Output contains invalid values")
            
    except Exception as e:
        print(f"✗ MPC solve failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test reset
    mpc.reset()
    print("✓ MPC reset successful")
    
    return True


def test_rl_mpc_integration():
    """Test RL-MPC integration scenario"""
    print("\n=== Testing RL-MPC Integration Scenario ===")
    
    # Simulate RL policy outputting weights
    print("Simulating RL policy output:")
    rl_weights = np.array([
        1.0, 1.5, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.1, 1.0, 1.0, 0.1, 0.0
    ])
    print(f"  Q weights from RL: {rl_weights}")
    
    # Create state
    state = mpc_controller.MPCState()
    state.root_pos = np.array([0.0, 0.0, 0.3])
    state.foot_pos = np.array([
        [0.17, 0.17, -0.17, -0.17],
        [0.15, -0.15, 0.15, -0.15],
        [-0.05, -0.05, -0.05, -0.05]
    ])
    state.contacts = [True, True, True, True]
    state.root_pos_d = np.array([0.0, 0.0, 0.3])
    state.root_lin_vel_d = np.array([0.3, 0.0, 0.0])
    
    # Create MPC and set weights
    mpc = mpc_controller.ConvexMpc()
    r_weights = np.ones(12) * 1e-5
    mpc.set_weights(rl_weights, r_weights)
    
    # Solve
    grf = mpc.solve(state)
    print(f"✓ Computed GRF with RL weights: {grf[:3]}... (showing first 3)")
    
    return True


def main():
    print("=" * 60)
    print("MPC Python Bindings Test")
    print("=" * 60)
    
    try:
        # Test MPCState
        state = test_mpc_state()
        
        # Test MPC Controller
        success = test_mpc_controller(state)
        
        if success:
            # Test RL-MPC integration
            test_rl_mpc_integration()
            
            print("\n" + "=" * 60)
            print("✓ All tests passed!")
            print("=" * 60)
            return 0
        else:
            print("\n" + "=" * 60)
            print("✗ Some tests failed")
            print("=" * 60)
            return 1
            
    except Exception as e:
        print(f"\n✗ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
