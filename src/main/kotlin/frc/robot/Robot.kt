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
import frc.robot.Constants

class Robot : TimedRobot() {
    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    val drivetrain = Drivetrain(
        Constants.Real.RIGHT_MOTOR_1,
        Constants.Real.RIGHT_MOTOR_2,
        Constants.Real.LEFT_MOTOR_1,
        Constants.Real.LEFT_MOTOR_2
    ) 

    val intake = brushlessMotor(Constants.Real.INTAKE)
    var intakeIsForward = false

    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    val turntable = brushlessMotor(Constants.Real.TURNTABLE)

    enum StickMode {
        TURNTABLE, SHOOTER
    }

    var rightMode = StickMode.TURNTABLE
    val xButtonLastPressed = false

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drivetrain.drive(speed, turn)

        val intakeSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.leftBumperPressed) intakeIsForward = !intakeIsForward
        intake.set(if (intakeIsForward) intakeSpeed else -intakeSpeed)

        if (controller.xButtonPressed)
            rightMode = when (rightMode) {
                StickMode.TURNTABLE -> StickMode.SHOOTER
                StickMode.SHOOTER -> StickMode.TURNTABLE
            }

        when (rightMode) {
            StickMode.TURNTABLE -> turntable.set(controller.rightY.pow(3.0))
            StickMode.SHOOTER -> {
                shooterTop.set(controller.rightX.pow(3.0))
                shooterBottom.set(controller.rightY.pow(3.0) / Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO)
            }
        }
    }

    override fun testInit() {}
    override fun testPeriodic() {
   }
}
