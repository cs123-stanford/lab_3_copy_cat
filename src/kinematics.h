#pragma once

#include <BasicLinearAlgebra.h>
#include "utils.h"

struct KinematicsConfig
{
    float l1;
    float l2;
    float l3;
};

// returns a rotation matrix about x according to angle theta
BLA::Matrix<4, 4> trans_rot_mat_x(double theta, double x, double y, double z)
{
    BLA::Matrix<4, 4> mat = {1, 0, 0, x,
                             0, cos(theta), -sin(theta), y,
                             0, sin(theta), cos(theta), z,
                             0, 0, 0, 1};
    return mat;
}

BLA::Matrix<4, 4> trans_rot_mat_y(double theta, double x, double y, double z)
{
    BLA::Matrix<4, 4> mat = {cos(theta), 0, sin(theta), x,
                             0, 1, 0, y,
                             -sin(theta), 0, cos(theta), z,
                             0, 0, 0, 1};
    return mat;
}

BLA::Matrix<4, 4> trans_rot_mat_z(double theta, double x, double y, double z)
{
    BLA::Matrix<4, 4> mat = {cos(theta), -sin(theta), 0, x,
                             sin(theta), cos(theta), 0, y,
                             0, 0, 1, z,
                             0, 0, 0, 1};
    return mat;
}

// TODO: Step 12. Implement forward kinematics
BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
    // /* Computes forward kinematics for the 3DOF robot arm/leg.

    // Returns the cartesian coordinates of the end-effector
    // corresponding to the given joint angles and leg configuration.
    // */
    BLA::Matrix<4, 1> end_effector_local = {0, 0, config.l3, 1};

    BLA::Matrix<4, 4> w_to_j1 = trans_rot_mat_z(joint_angles(0), 0.0, 0.0, 0.0);

    BLA::Matrix<4, 4> j1_to_j2 = trans_rot_mat_y(joint_angles(1), 0.0, config.l1, 0.0);

    BLA::Matrix<4, 4> j2_to_j3 = trans_rot_mat_y(-joint_angles(2), 0.0, 0.0, config.l2);

    BLA::Matrix<4, 1> homogeneous_coords = (w_to_j1 * (j1_to_j2 * (j2_to_j3 * end_effector_local)));

    BLA::Matrix<3, 1> cartesian_coords = {homogeneous_coords(0), homogeneous_coords(1), homogeneous_coords(2)};

    return cartesian_coords;
}

// Epsilon for Newton Method
const double EPS = 1e-2;
const double ALPHA = 0.1;
/**
 * Cost function for ik
 * Calculate the squared-norm error between the position returned by FK(guess) and the target location
 */
float distance(const BLA::Matrix<3> &target_location, const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
    // TODO: Write code here!
    return 0;
}
/**
 *  To implement the calculate_gradient function:
 *
 *  1. Calculate the gradient of the cost for each motor:
 *     a. Calculate the perturbed cost of taking an EPS step in the positive direction for each of the current joint angles.
 *     b. Calculate the nominal cost of being at the target_location from the current joint angles.
 *     c. Calculate the gradient by subtracting (cost_perturbed - cost_nominal) divided by epsilon (EPS).
 *  2. Multiply each gradient by a scaling factor ALPHA for gradient adjustment.
 *  3. Return a BLA::Matrix<3> to store the gradients for each motor
 */
BLA::Matrix<3> calculate_gradient(const BLA::Matrix<3> joint_angles, const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
    // TODO: Replace and add code here
    BLA::Matrix<3> gradient(0, 0, 0);
    return gradient;
}
/**
 *  Implement this function
 *
 *  1. Initialize joint_angles with cur_joint_angles to start the optimization process.
 *  2. Set current_cost to the initial distance between target_location and joint_angles using the provided distance function.
 *  3. Run a loop for a predefined number of iterations, (use 200 iterations).
 *     a. Calculate the cost gradient with respect to the joint angles using calculate_gradient function.
 *     b. Update joint_angles by taking a gradient step to minimize the cost.
 *     c. Recalculate the current_cost after the update.
 *  4. Return the optimized joint_angles.
 */
BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config, const BLA::Matrix<3> &cur_joint_angles)
{
    // TODO: Replace and add code here!
    BLA::Matrix<3> joint_angles = cur_joint_angles;

    return joint_angles;
}

enum class BodySide
{
    kLeft,
    kRight,
    kUnspecified
};
