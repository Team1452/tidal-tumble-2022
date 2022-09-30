#include "subsystems/Drivetrain.h"

Drivetrain &Drivetrain::GetDefault() {
    // if (m_instance == nullptr) {
    //     m_instance = new Drivetrain(); 
    // }
    // return m_instance;
}

void Drivetrain::DriveLeft(double speed) {
    // m_leftMotor->Set(speed);
}

void Drivetrain::DriveRight(double speed) {
    // m_rightMotor->Set(speed);
}

Drivetrain::Drivetrain() {
    // m_leftMotor = new rev::CANSparkMax(Constants::Test::Drivetrain::LEFT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    // m_rightMotor = new rev::CANSparkMax(Constants::Test::Drivetrain::RIGHT_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

    // m_leftMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // m_rightMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // m_controller = new frc::XboxController(0);
}
