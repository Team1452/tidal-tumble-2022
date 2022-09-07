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

#include <thread>
#include <chrono>
#include <iostream>

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
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
  // Cycle between forward/reverse for testing piston (every period or 1000ms)
  if (m_pistonForward) {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
  } else {
    m_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  }
  m_pistonForward = !m_pistonForward;
}

void Robot::TestInit() {
  std::cout << "Starting test!\n";

  m_solenoid = new frc::DoubleSolenoid{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  m_pistonForward = true;
  
  std::cout << "Initialized!\n";
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
