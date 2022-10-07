package frc.robot

import kotlin.math.*

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.networktables.NetworkTableInstance
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import com.ctre.phoenix.sensors.Pigeon2
import io.javalin.Javalin

import frc.robot.Drivetrain

class Robot : TimedRobot() {
    companion object {
        val RIGHT_MOTOR_1 = 1
        val RIGHT_MOTOR_2 = 1

        val LEFT_MOTOR_1 = 15
        val LEFT_MOTOR_2 = 6

        val INTAKE = 14
        val CLIMB = 7

        // TODO
        val SHOOTER_BOTTOM = 5
        val SHOOTER_TOP = 6
    }

    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    val drivetrain = Drivetrain(LEFT_MOTOR_1, LEFT_MOTOR_2, RIGHT_MOTOR_1, RIGHT_MOTOR_2)

    val intake = brushlessMotor(INTAKE)
    var intakeIsForward = false

    val shooterTop = brushlessMotor(SHOOTER_TOP)
    val shooterBottom = brushlessMotor(SHOOTER_BOTTOM)

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drivetrain.drive(speed, turn)

        val intakeSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.leftBumperPressed) intakeIsForward = !intakeIsForward
        intake.set(if (intakeIsForward) intakeSpeed else -intakeSpeed)

        shooterBottom.set(controller.rightY.pow(3.0))
        shooterTop.set(controller.rightX.pow(3.0))
    }

    override fun testInit() {}
    override fun testPeriodic() {
   }
}
