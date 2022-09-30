// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsBase.h>
#include <frc/PneumaticHub.h>
#include <frc/I2C.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>
#include <ctre/phoenix/sensors/Pigeon2.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <bitset>
#include <cmath>

using namespace std;

Robot::Robot() : frc::TimedRobot(20_ms) {
  m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  m_leftMotor = new rev::CANSparkMax(Constants::Test::Drivetrain::LEFT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
  m_rightMotor = new rev::CANSparkMax(Constants::Test::Drivetrain::RIGHT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

  m_leftMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_controller = new frc::XboxController(0);

  m_pigeon2 = new ctre::phoenix::sensors::Pigeon2(10);
}

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
  cout << m_controller->GetLeftY() << ", " << m_controller->GetLeftX() << endl;
  double threshold = 0.01;
  double speed = pow((m_controller->GetLeftY() - threshold) * 1/(1-threshold), 3.0);
  double turn = pow((m_controller->GetLeftX() - threshold) * 1/(1-threshold), 3.0);

  cout << "speed: " << speed << ", turn: " << turn << endl;
  m_leftMotor->Set(-speed + turn);
  m_rightMotor->Set(speed + turn);
}
/*
void Robot::TeleopExit() {
  delete m_leftMotor;
  delete m_rightMotor;
  delete m_controller;
}
*/
/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
  double yaw = m_pigeon2->GetYaw();
  double pitch = m_pigeon2->GetPitch();
  double roll = m_pigeon2->GetRoll();

  cout << "Yaw: " << yaw << "; pitch: " << pitch << "; roll: " << roll << endl;

  int16_t buf[3];

  m_pigeon2->GetBiasedAccelerometer((int16_t*)&buf);

  cout << "Accelerometer: " << buf[0] << ", " << buf[1] << ", " << buf[2] << endl;

  double targetOffsetAngle_Horizontal = m_table->GetNumber("tx", 0.0);
  double targetOffsetAngle_Vertical = m_table->GetNumber("ty", 0.0);
  double targetArea = m_table->GetNumber("ta", 0.0);
  double targetSkew = m_table->GetNumber("ts", 0.0);

  double targetAngle;

  if (targetArea > 0.05) {
    targetAngle = targetOffsetAngle_Horizontal;
    m_lastOffsetAngleHorizontal = targetOffsetAngle_Horizontal;
  } else {
    targetAngle = m_lastOffsetAngleHorizontal;
  }

  double turn = clamp(targetAngle / 50, -0.1, 0.1);
  // turn = abs(turn) > 0.06 ? turn : 0;

  m_leftMotor->Set(turn);
  m_rightMotor->Set(turn);

  cout << "Turn: " << turn << endl;

  cout << "target angle offset: " 
    << targetOffsetAngle_Horizontal
    << ", "
    << targetOffsetAngle_Vertical
    << "; target area: "
    << targetArea
    << "; target skew: "
    << targetSkew
    << endl;
}

void Robot::TestInit() {
  // auto entries = nt::NetworkTableInstance::GetDefault().GetEntries("", 0);
  // for (auto entry : entries) {
  //   cout << entry.GetName() << "; type: " << (int)entry.GetType() << endl;
  // }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
