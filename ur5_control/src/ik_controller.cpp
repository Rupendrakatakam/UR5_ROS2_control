#include "ur5_control/ik_controller.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

bool IKController::init(std::string robot_urdf, std::string shoulder_link, std::string hand_link) {
    
    // Step 1 :read the URDF
    KDL::Tree robot_body_tree;
    if (!kdl_parser::treeFromString(robot_urdf, robot_body_tree)) {
        std::cerr << "I couldn't read the robot urdf." << std::endl;
        return false; 
    }

    // Step 2: Trace the path from the shoulder to the hand
    if (!robot_body_tree.getChain(shoulder_link, hand_link, my_arm_)) {
        std::cerr << "I can't find a path from the shoulder to the hand!" << std::endl;
        return false;
    }

    // Step 3: Turn on my three main calculators
    // Calculator A: "Where is my hand?"
    forward_math_calculator_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(my_arm_);
    
    // Calculator B: "How fast should I spin my motors?"
    speed_math_calculator_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(my_arm_);
    
    // Calculator C: "How do I reach that target?" (Needs A and B to work)
    reach_target_calculator_ = std::make_shared<KDL::ChainIkSolverPos_NR>(my_arm_, *forward_math_calculator_, *speed_math_calculator_, 100, 1e-6);

    std::cout << "I am awake! I counted my joints and I have: " << my_arm_.getNrOfJoints() << std::endl;
    return true; // Waking up was a success!
}

// Forward Kinematics
bool IKController::computeFK(std::vector<double> current_motor_angles, KDL::Frame& answer_hand_position) {
    
    // Convert our standard list of numbers into a special format the calculator likes
    KDL::JntArray calculator_format_angles(my_arm_.getNrOfJoints());
    for (unsigned int i = 0; i < my_arm_.getNrOfJoints(); i++) {
        calculator_format_angles(i) = current_motor_angles[i];
    }

    // Ask Calculator A to figure out where the hand is
    int success_score = forward_math_calculator_->JntToCart(calculator_format_angles, answer_hand_position);
    
    // If the score is 0 or higher, we did the math correctly!
    return (success_score >= 0);
}


// Inverse Kinematics
bool IKController::computeIK(KDL::Frame target_goal, std::vector<double> current_motor_angles, std::vector<double>& answer_new_motor_angles) {
    
    // Prepare the calculator's input (where we are right now) and output (the blank answer sheet)
    KDL::JntArray starting_guess(my_arm_.getNrOfJoints());
    KDL::JntArray blank_answer_sheet(my_arm_.getNrOfJoints());

    for (unsigned int i = 0; i < my_arm_.getNrOfJoints(); i++) {
        starting_guess(i) = current_motor_angles[i];
    }

    // Ask Calculator C to figure out how to reach the target
    int success_score = reach_target_calculator_->CartToJnt(starting_guess, target_goal, blank_answer_sheet);

    // Did the calculator find an answer?
    if (success_score >= 0) {
        // Yes! Copy the answers from the sheet back to our main list
        answer_new_motor_angles.resize(my_arm_.getNrOfJoints());
        for (unsigned int i = 0; i < my_arm_.getNrOfJoints(); i++) {
            answer_new_motor_angles[i] = blank_answer_sheet(i);
        }
        return true; 
    }
    
    // No... the target is too far away or behind a wall!
    return false; 
}


// Velocity Control
bool IKController::computeJointVelocities(std::vector<double> current_motor_angles, std::vector<double> desired_hand_speed, std::vector<double>& answer_motor_speeds) {
    
    // Put current angles into calculator format
    KDL::JntArray starting_angles(my_arm_.getNrOfJoints());
    for (unsigned int i = 0; i < my_arm_.getNrOfJoints(); i++) {
        starting_angles(i) = current_motor_angles[i];
    }

    // Tell the calculator how fast we want the hand to move (X, Y, Z speed and twisting speed)
    KDL::Twist speed_target;
    speed_target.vel = KDL::Vector(desired_hand_speed[0], desired_hand_speed[1], desired_hand_speed[2]);
    speed_target.rot = KDL::Vector(desired_hand_speed[3], desired_hand_speed[4], desired_hand_speed[5]);

    // Create a blank sheet for the motor speed answers
    KDL::JntArray blank_speed_answers(my_arm_.getNrOfJoints());

    // Ask Calculator B to figure out the motor speeds
    int success_score = speed_math_calculator_->CartToJnt(starting_angles, speed_target, blank_speed_answers);

    // If we found the answer, copy it to our main list
    if (success_score >= 0) {
        answer_motor_speeds.resize(my_arm_.getNrOfJoints());
        for (unsigned int i = 0; i < my_arm_.getNrOfJoints(); i++) {
            answer_motor_speeds[i] = blank_speed_answers(i);
        }
        return true;
    }
    
    return false; // Math failed
}