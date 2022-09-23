// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsBase.h>
#include <frc/PneumaticHub.h>
#include <frc/I2C.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <bitset>
#include <cmath>

using namespace std;

// Period every 1000ms for testing
Robot::Robot() : frc::TimedRobot(1000_ms) {}

void Robot::RobotInit() {
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // TODO: Update firmware and put device IDs in Constants.h + move to Drivetrain
  m_leftMotor = new rev::CANSparkMax(0, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
  m_rightMotor = new rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

  m_controller = new frc::XboxController(0); // Should be in Constants.h


  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // TODO: Figure out actual controller scheme
  m_leftMotor->Set(pow(m_controller->GetLeftTriggerAxis(), 2.0));
  m_rightMotor->Set(pow(m_controller->GetRightTriggerAxis(), 2.0));
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
  // if (m_solenoid != nullptr) {
  //   // Cycle between forward/reverse for testing piston (every period or 1000ms)
  //   if (m_pistonForward) {
  //     cout << "Driving forward: ";
  //     m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   } else {
  //     cout << "Driving reverse: ";
  //     m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  //   }
  //   cout << "Solenoid: " << m_solenoid->Get() << '\n';
  //   m_pistonForward = !m_pistonForward;
  // }
}

void Robot::TestInit() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
