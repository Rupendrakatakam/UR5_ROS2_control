#include "ur5_control/ik_controller.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

IKController::IKController() {}

IKController::~IKController() {}

bool IKController::init(const std::string& urdf_string, const std::string& base_link, const std::string& tip_link) {
    KDL::Tree my_tree;
    
    // Parse the URDF string into a KDL Tree
    if (!kdl_parser::treeFromString(urdf_string, my_tree)) {
        std::cerr << "Failed to construct KDL tree from URDF string." << std::endl;
        return false;
    }

    // Extract the kinematic chain for the UR5e (e.g., from 'base_link' to 'tool0')
    if (!my_tree.getChain(base_link, tip_link, chain_)) {
        std::cerr << "Failed to get KDL chain from " << base_link << " to " << tip_link << std::endl;
        return false;
    }

    // Initialize the KDL Solvers
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    
    // Inverse velocity solver (uses pseudo-inverse, great for Task 2)
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
    
    // Inverse position solver (uses Newton-Raphson, great for Task 1)
    // Needs FK and IK_Vel solvers to work. Max 100 iterations, 1e-6 tolerance.
    ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);
    
    // Jacobian solver
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

    std::cout << "IK Controller initialized successfully. Joints in chain: " << chain_.getNrOfJoints() << std::endl;
    return true;
}

unsigned int IKController::getNumJoints() const {
    return chain_.getNrOfJoints();
}

bool IKController::computeFK(const std::vector<double>& joint_positions, KDL::Frame& end_effector_pose) {
    if (joint_positions.size() != chain_.getNrOfJoints()) return false;

    KDL::JntArray q(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
        q(i) = joint_positions[i];
    }

    // Calculate FK
    int status = fk_solver_->JntToCart(q, end_effector_pose);
    return (status >= 0);
}

bool IKController::computeIK(const KDL::Frame& desired_pose, const std::vector<double>& current_joints, std::vector<double>& desired_joints) {
    if (current_joints.size() != chain_.getNrOfJoints()) return false;

    KDL::JntArray q_init(chain_.getNrOfJoints());
    KDL::JntArray q_out(chain_.getNrOfJoints());
    
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
        q_init(i) = current_joints[i];
    }

    // Calculate IK
    int status = ik_pos_solver_->CartToJnt(q_init, desired_pose, q_out);
    
    if (status >= 0) {
        desired_joints.resize(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            desired_joints[i] = q_out(i);
        }
        return true;
    }
    return false; // IK Failed (e.g., unreachable pose or singularity)
}

bool IKController::computeJointVelocities(const std::vector<double>& current_joints, const std::vector<double>& cartesian_velocities, std::vector<double>& joint_velocities) {
    // cartesian_velocities should be size 6: [vx, vy, vz, wx, wy, wz]
    if (current_joints.size() != chain_.getNrOfJoints() || cartesian_velocities.size() != 6) return false;

    KDL::JntArray q_in(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
        q_in(i) = current_joints[i];
    }

    // Create KDL Twist (Cartesian velocity)
    KDL::Twist v_in;
    v_in.vel = KDL::Vector(cartesian_velocities[0], cartesian_velocities[1], cartesian_velocities[2]);
    v_in.rot = KDL::Vector(cartesian_velocities[3], cartesian_velocities[4], cartesian_velocities[5]);

    KDL::JntArray qdot_out(chain_.getNrOfJoints());

    // Calculate Joint Velocities using Pseudo-Inverse Jacobian
    int status = ik_vel_solver_->CartToJnt(q_in, v_in, qdot_out);

    if (status >= 0) {
        joint_velocities.resize(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
            joint_velocities[i] = qdot_out(i);
        }
        return true;
    }
    return false;
}