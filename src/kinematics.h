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
BLA::Matrix<3, 3> rot_mat_x(float theta)
{
  BLA::Matrix<3, 3> rot_x = {1,
                             0,
                             0,
                             0,
                             cos(theta),
                             -sin(theta),
                             0,
                             sin(theta),
                             cos(theta)};
  return rot_x;
}

// returns a rotation matrix about y according to angle theta
BLA::Matrix<3, 3> rot_mat_y(float theta)
{
  BLA::Matrix<3, 3> rot_y = {cos(theta),
                             0,
                             sin(theta),
                             0,
                             1,
                             0,
                             -sin(theta),
                             0,
                             cos(theta)};

  return rot_y;
}

// returns a rotation matrix about z according to angle theta
BLA::Matrix<3, 3> rot_mat_z(float theta)
{
  BLA::Matrix<3, 3> rot_z = {cos(theta),
                             -sin(theta),
                             0,
                             sin(theta),
                             cos(theta),
                             0,
                             0,
                             0,
                             1};
  return rot_z;
}

BLA::Matrix<3> forward_kinematics(const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /* Computes forward kinematics for the 3DOF robot arm/leg.

  Returns the cartesian coordinates of the end-effector
  corresponding to the given joint angles and leg configuration.
  */

  // TODO: Fill in from lab 2!
  return BLA::Matrix<3>(0, 0, 0);
}

// Epsilon for Gradient Method
const double EPS = 1e-2;
const double IK_THRESH = 0.001;

float cost(const BLA::Matrix<3> &target_location, const BLA::Matrix<3> &joint_angles, const KinematicsConfig &config)
{
  /**
   * Cost function for ik
   * Calculate the squared-norm error between the position returned by FK(guess) and the target location
   */
  /** FILL ME IN */
  return 0;
}

BLA::Matrix<3> calculate_cost_gradient(const BLA::Matrix<3> joint_angles, const BLA::Matrix<3> &target_location, const KinematicsConfig &config)
{
  /**
   * Calculate the gradient of the cost
   *        1. Calculate the pertubed cost of taking a EPS step in the positive direction for each of the current joint angles
   *        2. Calculate the nominal cost of being at the target_location from the joint angles
   *        3. Calculate the gradient by subtracting the (cost_pertubed - cost_nominal) / epsilon
   */
  /** FILL ME IN */
  return BLA::Matrix<3>(0, 0, 0);
}

BLA::Matrix<3> inverse_kinematics(const BLA::Matrix<3> &target_location, const KinematicsConfig &config, const BLA::Matrix<3> &cur_joint_angles)
{
  BLA::Matrix<3> joint_angles = cur_joint_angles;

  for (int i = 0; i < 200; i++)
  {
    /**
     *       1. Calculate the cost gradient for the target location with respect to the joint angles
     *       2. Take a gradient step for each of the joint angles
     *       3. Calculate the current cost and if it is less than or equal to IK_THRESH, then break
     */
    /** FILL ME IN */
  }
  return joint_angles;
}

enum class BodySide
{
  kLeft,
  kRight,
  kUnspecified
};