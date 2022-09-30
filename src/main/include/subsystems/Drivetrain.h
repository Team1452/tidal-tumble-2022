#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <frc/XboxController.h>

class Drivetrain {
public:
  Drivetrain &GetDefault();

  void DriveLeft(double speed);
  void DriveRight(double speed);

  void DifferentialDrive(double speed, double turn);

private:
  Drivetrain();

//   static Drivetrain &m_instance;

//   rev::CANSparkMax &m_leftMotor;
//   rev::CANSparkMax &m_rightMotor;
//   frc::XboxController &m_controller;
};

