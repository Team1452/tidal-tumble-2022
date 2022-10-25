package frc.robot

import kotlin.math.*

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.networktables.NetworkTableInstance
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.IOException
import java.io.FileWriter
import com.ctre.phoenix.sensors.Pigeon2
import io.javalin.Javalin
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.MotorType.*
import com.revrobotics.CANSparkMaxLowLevel.*
import com.opencsv.CSVWriter
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory

import frc.robot.Drivetrain
import frc.robot.Constants
import frc.robot.main

fun Double.deadzoneOne(threshold: Double): Double = sign(this) * max(0.0, abs(this) - threshold) / (1.0-threshold)

class Robot : TimedRobot(Constants.PERIOD) {
    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    val drive = Drivetrain(
        Constants.Real.LEFT_MOTOR_1,
        Constants.Real.LEFT_MOTOR_2,
        Constants.Real.RIGHT_MOTOR_1,
        Constants.Real.RIGHT_MOTOR_2
    )
    // val drive = DriveSubsystem(
    //     listOf(
    //         CANSparkMax(Constants.Real.LEFT_MOTOR_1, kBrushless),
    //         CANSparkMax(Constants.Real.LEFT_MOTOR_2, kBrushless)
    //     ),
    //     listOf(
    //         CANSparkMax(Constants.Real.RIGHT_MOTOR_1, kBrushless),
    //         CANSparkMax(Constants.Real.RIGHT_MOTOR_2, kBrushless)
    //     ),
    //     // Pigeon2(Constants.Real.PIGEON)
    // )

    // var writer = CSVWriter(FileWriter(Constants.Real.CSV_PATH, true); 
    var writer = FileWriter(Constants.Real.CSV_PATH, true)

    fun log(dist: Double, shooterSpeed: Double?) {
        writer.write("$dist,$shooterSpeed")
        writer.flush()
    }

    var bottomSpeed = 0.0
    var upperSpeed = 0.0

    var app = Javalin.create().apply {
        ws("/shooter") { ws -> 
            ws.onConnect { ctx -> 
                println("New websocket connected: $ctx")
            }
            ws.onClose { ctx -> 
                println("Websocket closed: $ctx")
            }
            ws.onMessage { ctx -> 
                bottomSpeed = ctx.message().substringBefore(',').toDouble()
                upperSpeed = ctx.message().substringAfter(',').toDouble()
                // ctx.session.remote.sendString("Echo: ${ctx.message()}")
            }
        }
    }.start(7070)

    val intake = brushlessMotor(Constants.Real.INTAKE)
    var intakeIsForward = false

    val colon = brushlessMotor(Constants.Real.COLON)
    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    val turntable = brushlessMotor(Constants.Real.TURNTABLE)

    var centering = false
    val xButtonLastPressed = false
    var ratio: Double = Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO 
    val scheduler = Scheduler()
    val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Real.LEFT_SOLENOID_1, Constants.Real.LEFT_SOLENOID_2);
    val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Real.RIGHT_SOLENOID_1, Constants.Real.RIGHT_SOLENOID_2);

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()

    override fun robotInit() {

    }

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        // scheduler.tick()

        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drive.drive(speed, turn)

        val intakeSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.aButton) intakeIsForward = !intakeIsForward
        intake.set(if (intakeIsForward) intakeSpeed else -intakeSpeed)

        // shooterTop.set((-controller.rightY.pow(3.0)) * (1.0 - ratio)))
        // shooterBottom.set((-controller.rightY.pow(3.0)) * ratio))
        shooterTop.set(-bottomSpeed)
        shooterBottom.set(-upperSpeed)
        turntable.set(controller.rightX.deadzoneOne(0.1))

        if (controller.yButtonPressed) centering = !centering
        if (controller.rightBumperPressed) centering = !centering

        if (controller.bButton)
            colon.set(-1.0)
        else
            colon.set(0.0)

        println("left bumper: ${controller.leftBumperPressed}, ${controller.leftBumperReleased}")
        if (controller.rightBumper) {
            println("Opening left/right solenoids")
            leftSolenoid.set(kForward)
            rightSolenoid.set(kForward)
        } else if (controller.leftBumperReleased) {
            println("Closing left/right solenoids")
            leftSolenoid.set(kReverse)
            rightSolenoid.set(kReverse)
        }

        if (centering) {
            val tv = limelightTable.getEntry("tv").getDouble(0.0)
            val tx = limelightTable.getEntry("tx").getDouble(0.0)

            if (tv == 1.0) {
                val turntableTurn = tv * (tx / 29.8 * 0.5).let { it.deadzoneOne(0.01) }
                turntable.set(turntableTurn)
            } else {
                // TODO: Naive radar sweep around 210 deg range if target not found 
                // turntable.set(-turntable.encoder.position / 30)
            }
        }
    }

    override fun testInit() {
        // try {
        //     val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        // } catch (err: IOException) {
        //     error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        // }

        // val ramseteCommand = RamseteCommand(
        //     trajectory,
        //     drive::getPose,
        //     RamseteController(Auto.RAMSETE_B, Constants.Auto.RAMSETE_B),
        //     SimpleMotorFeedforward(
        //         KS_VOLTS,
        //         KV_VOLT_SECONDS_PER_METER,
        //         KA_VOLT_SECONDS_SQUARED_PER_METER
        //     ),
        //     DRIVE_KINEMATICS,
        //     drive::getWheelSpeeds,
        //     PIDController(P_DRIVE_VEL, 0.0, 0.0),
        //     PIDController(P_DRIVE_VEL, 0.0, 0.0),
        //     drive::tankDriveVolts,
        //     drive
        // )

        // drive.resetOdometry(trajectory.initialPose)

        // ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
    }
    override fun testPeriodic() {
        CommandScheduler.getInstance().run()
    }
}