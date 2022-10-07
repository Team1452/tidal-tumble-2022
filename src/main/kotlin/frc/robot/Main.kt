package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier

import frc.robot.Robot

fun main() {
    RobotBase.startRobot({ Robot() })
}