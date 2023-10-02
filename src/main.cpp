#include "Arduino.h"
#include "C610Bus.h"
#include <array>
#include "BasicLinearAlgebra.h"
#include "pid.h"
#include "kinematics.h"
#include "test_kinematics.h"
#include "test_inv_kinematics.h"
#include "utils.h"

#define DO_TESTS = // comment out to skip running tests before executing main program

/*********** STUDENT CONFIGURATION NECESSARY *************/
// Examine your robot leg to see if you built a left leg or a right leg.
// Then replace kUnspecified with the correct side.
const BodySide kLegSide = BodySide::kRight;
/*********************************************************/

long last_command = 0; // To keep track of when we last commanded the motors
C610Bus<CAN2> bus2;    // Initialize the Teensy's CAN bus to talk to the rear Pupper motors
C610Bus<CAN1> bus1;

const int LOOP_DELAY_MILLIS = 5; // Wait for 0.005s between motor updates.

const float Kp = 2000; // 2000;
const float Kd = 100;  // 100;
const float kMaxCurrent = 3000;

// Define the signed hip offset and link lengths
const KinematicsConfig pupper_leg_config = (kLegSide == BodySide::kLeft) ? KinematicsConfig{0.035, 0.08, 0.11} : KinematicsConfig{-0.035, 0.08, 0.11};

BLA::Matrix<3> actuator_angles2{0, 0, 0};     // rad
BLA::Matrix<3> actuator_velocities2{0, 0, 0}; // rad/s
BLA::Matrix<3> actuator_commands2{0, 0, 0};   // mA

BLA::Matrix<3> actuator_angles1{0, 0, 0};     // rad
BLA::Matrix<3> actuator_velocities1{0, 0, 0}; // rad/s
BLA::Matrix<3> actuator_commands1{0, 0, 0};   // mA
void setup()
{
  clear_serial_buffer();
  wait_for_key('s', "Remember to start the arm extended out horizontally.");
#ifdef DO_TESTS
  test_inv_kinematics();
#endif
}

float flip = 1.0;

void loop()
{
  bus2.PollCAN(); // Check for messages from the motors.
  bus1.PollCAN();
  long now = millis();

  // Check to see if we received a 's' and if so, stop the program.
  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus2.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      bus1.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  // Check to see if it's time to run our control loop again.
  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    for (int i = 0; i < 3; i++)
    {
      actuator_angles2(i) = bus2.Get(i).Position();
      actuator_velocities2(i) = bus2.Get(i).Velocity();
      actuator_angles1(i) = bus1.Get(i).Position();
      actuator_velocities1(i) = bus1.Get(i).Velocity();
    }

    BLA::Matrix<3> cartesian_coordinates2 = forward_kinematics(actuator_angles2, pupper_leg_config);

    // BLA::Matrix<3> test_point(0.1, -0.07, 0);
    // 0.16 m
    cartesian_coordinates2(1) += 0.16;
    BLA::Matrix<3> angles1 = inverse_kinematics(cartesian_coordinates2, pupper_leg_config, actuator_angles1);
    actuator_commands1 = vectorized_pd(actuator_angles1, actuator_velocities1, angles1, Kp, Kd);
    // TODO: Uncomment to test a reachable test point to check IK-FK is correct
    // BLA::Matrix<3> test_point(-0.1, 0.07, 0);
    // BLA::Matrix<3> angles = inverse_kinematics(test_point, pupper_leg_config, actuator_angles1);
    // print_vector(forward_kinematics(angles, pupper_leg_config));

    actuator_commands1 = vectorized_sanitize(actuator_commands1,
                                             actuator_angles1,
                                             actuator_velocities1,
                                             kMaxCurrent);

    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.
    bus1.CommandTorques(actuator_commands1(0), actuator_commands1(1), actuator_commands1(2), 0, C610Subbus::kOneToFourBlinks);

    last_command = now;
    Serial.println();
  }
}