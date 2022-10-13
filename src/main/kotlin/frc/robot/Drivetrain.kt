/*
package frc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.*
import frc.robot.main

fun brushlessMotor(port: Int) = CANSparkMax(port, MotorType.kBrushless)

class Drivetrain(val leftPort: Int, val left2Port: Int?, val rightPort: Int, val right2Port: Int?) {
    val left = brushlessMotor(leftPort)
    val left2 = if (left2Port != null) brushlessMotor(left2Port) else null
    val right = brushlessMotor(rightPort)
    val right2 = if (right2Port != null) brushlessMotor(right2Port) else null

    constructor(leftPort: Int, rightPort: Int) : this(leftPort, null, rightPort, null)

    fun driveLeft(speed: Double) {
        left.set(speed)
        left2?.set(speed)
    }

    fun driveRight(speed: Double) {
        right.set(speed)
        right2?.set(speed)
    }

    fun drive(speed: Double, turn: Double) {
        driveLeft(speed + turn)
        driveRight(speed - turn)
    }
}
 */