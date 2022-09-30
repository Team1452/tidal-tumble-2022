// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <networktables/NetworkTable.h>
#include <ctre/phoenix/sensors/Pigeon2.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  //void TeleopExit() override;
  void TestExit() override;
  void TestPeriodic() override;
  void TestInit() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;
  frc::DoubleSolenoid* m_solenoid = nullptr;
  bool m_pistonForward = false;

  double m_targetDeltaX = 0;
  double m_lastOffsetAngleHorizontal = 0;

  // TODO: Move to Drivetrain
  rev::CANSparkMax *m_leftMotor;
  rev::CANSparkMax *m_rightMotor;

  frc::XboxController *m_controller;

  std::shared_ptr<nt::NetworkTable> m_table;

  ctre::phoenix::sensors::Pigeon2 *m_pigeon2;

  RobotContainer m_container;
};
