/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/
#pragma once
#include <kinematics.h>
#include <utils.h>
void test_ik()
{
  const KinematicsConfig pupper_leg_config = KinematicsConfig{-0.035, 0.08, 0.11};
  BLA::Matrix<3> actuator_angles1(PI / 2, PI / 2, PI / 2);
  /**
   1. Create a reachable test point
   2. Calculate the inverse kinematics solution to get the motor angles
   3. Verify the motor angles are the same as the test point using assert_close from utils.h with eps = 0.01
   */
   /** TODO: Fill me in! **/
}

void test_inv_kinematics()
{
  test_ik();
}
