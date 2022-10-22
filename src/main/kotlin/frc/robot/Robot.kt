package frc.robot

import kotlin.math.*

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.IOException
import com.ctre.phoenix.sensors.Pigeon2
import io.javalin.Javalin

import frc.robot.Drivetrain
import frc.robot.Constants

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.*
import frc.robot.main


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

    enum class StickMode {
        TURNTABLE, SHOOTER
    }

    var rightMode = StickMode.TURNTABLE
    val xButtonLastPressed = false

    var topSpeed:NetworkTableEntry? = null;
    var bottomSpeed:NetworkTableEntry? = null;
    var saveButton:NetworkTableEntry? = null;
    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory:Trajectory? = Trajectory()

    override fun teleopInit() {

    }
    override fun teleopPeriodic() {
        val trajectory = null
        try {
            val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)

        } catch (err: IOException) {
            error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        }
        val tab = Shuffleboard.getTab("Test");
        topSpeed = tab.add("topSpeed", 1).withWidget(BuiltInWidgets.kNumberSlider)
                   .getEntry();
        bottomSpeed =
                tab.add("bottomSpeed", 2).withWidget(BuiltInWidgets.kNumberSlider)
                   .getEntry();

        saveButton =
                tab.add("Save Speed", 3).withWidget(BuiltInWidgets.kBooleanBox).getEntry()
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
            var ratio = controller.rightY.pow(3.0) / 2.5
        when (rightMode) {
            StickMode.TURNTABLE -> {//turntable.set(controller.rightX)
                shooterTop.set(-controller.rightY.pow(3.0)) 
                shooterBottom.set((-controller.rightY.pow(3.0)) / Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO)
                println(controller.rightY)
        }
            StickMode.SHOOTER -> {
                shooterTop.set(controller.rightTriggerAxis.pow(3.0))
                shooterBottom.set(controller.rightTriggerAxis.pow(3.0) / ratio)
            }
        }
        drivetrain.drive(speed, turn)
        shooterTop.set(topSpeed!!.getDouble(0.0));
        shooterBottom.set(bottomSpeed!!.getDouble(0.0));
        if(saveButton!!.getBoolean(false)){
            saveButton!!.forceSetBoolean(false)
        }
    }

}
