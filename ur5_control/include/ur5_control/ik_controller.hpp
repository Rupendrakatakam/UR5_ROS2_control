#ifndef UR5_CONTROL_IK_CONTROLLER_HPP
#define UR5_CONTROL_IK_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>

// KDL Libraries for Kinematics
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

class IKController {
public:
    IKController();
    ~IKController();

    // 1. Initialize the KDL tree and solvers from the URDF string
    bool init(const std::string& urdf_string, const std::string& base_link, const std::string& tip_link);

    // 2. Forward Kinematics: Get Cartesian Pose from Joint Angles
    bool computeFK(const std::vector<double>& joint_positions, KDL::Frame& end_effector_pose);

    // 3. Inverse Kinematics (Task 1): Get Joint Angles from Cartesian Pose
    bool computeIK(const KDL::Frame& desired_pose, const std::vector<double>& current_joints, std::vector<double>& desired_joints);

    // 4. Inverse Velocity Kinematics (Task 2): Convert Cartesian Velocity to Joint Velocity
    bool computeJointVelocities(const std::vector<double>& current_joints, const std::vector<double>& cartesian_velocities, std::vector<double>& joint_velocities);

    // Helper: Get the number of joints in the chain
    unsigned int getNumJoints() const;

private:
    KDL::Chain chain_;
    
    // Pointers to the KDL solvers
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;  // Pseudo-inverse for Task 2
    std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;    // Newton-Raphson for Task 1
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
};

#endif // UR5_CONTROL_IK_CONTROLLER_HPP