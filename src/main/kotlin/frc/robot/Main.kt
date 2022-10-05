// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot

import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier

fun main() {
    RobotBase.startRobot(::Robot as Supplier<RobotBase!>!)
}