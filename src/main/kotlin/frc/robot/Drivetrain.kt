package robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel

class Drivetrain(val leftPort: Int, val rightPort: Int) {
    val left = CANSparkMax(leftPort, CANSparkMaxLowLevel.MotorType.kBrushless)
    val right = CANSparkMax(rightPort, CANSparkMaxLowLevel.MotorType.kBrushless)

    fun driveLeft(speed: Double) = left.set(speed)
    fun driveRight(speed: Double) = right.set(speed)
    fun drive(speed: Double, turn: Double) {
        driveLeft(speed + turn)
        driveRight(speed - turn)
    }
}