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
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Compressor
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
fun Double.clamp(minValue: Double, maxValue: Double): Double = min(max(this, minValue), maxValue)

enum class Direction {
    FORWARD, BACKWARD;

    fun toggle(): Direction = when (this) {
        FORWARD -> BACKWARD
        BACKWARD -> FORWARD
    }

    fun sign(): Double = when (this) {
        FORWARD -> 1.0
        BACKWARD -> -1.0
    }
}

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
    // var writer = FileWriter(Filesystem.getDeployDirectory().toPath().resolve(Constants.Real.CSV_PATH).toString())
    var topSpeed = 0.0
    var bottomSpeed = 0.0

    // var app = Javalin.create().apply {
    //     ws("/shooter") { ws -> 
    //         ws.onConnect { ctx -> 
    //             println("New websocket connected: $ctx")
    //         }
    //         ws.onClose { ctx -> 
    //             println("Websocket closed: $ctx")
    //         }
    //         ws.onMessage { ctx -> 
    //             bottomSpeed = ctx.message().substringBefore(',').toDouble()
    //             topSpeed = ctx.message().substringAfter(',').toDouble()
    //             ctx.session.remote.sendString("CONFIG_UPDATED")
    //         }
    //     }
    // }.start(7070)

    val intake = brushlessMotor(Constants.Real.INTAKE)
    var intakeDirection = Direction.FORWARD

    val colon = brushlessMotor(Constants.Real.COLON)

    val climb = brushlessMotor(Constants.Real.CLIMB)
    var climbDirection = Direction.FORWARD

    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    val turntable = brushlessMotor(Constants.Real.TURNTABLE)

    var centering = false
    val xButtonLastPressed = false
    var ratio: Double = Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO 
    val scheduler = Scheduler()
    val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Real.LEFT_SOLENOID_1, Constants.Real.LEFT_SOLENOID_2);
    // val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Real.RIGHT_SOLENOID_1, Constants.Real.RIGHT_SOLENOID_2);

    var topSpeedSlider: NetworkTableEntry? = null;
    var bottomSpeedSlider: NetworkTableEntry? = null;
    var saveButton: NetworkTableEntry? = null;

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()

    var speedConfigMode = false

    val DRIVE_ENABLED = false

    val pcmCompressor = Compressor(0, PneumaticsModuleType.CTREPCM)

    var ticks = 0;

    override fun robotInit() {
        pcmCompressor.disable()

        val tab = Shuffleboard.getTab("Test");
        topSpeedSlider = tab.add("topSpeed", 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        bottomSpeedSlider = tab.add("bottomSpeed", 2).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
        saveButton = tab.add("Save Speed", 3).withWidget(BuiltInWidgets.kBooleanBox).getEntry()
    }

    override fun teleopInit() { }
    override fun teleopPeriodic() {
        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        if (DRIVE_ENABLED)
            drive.drive(speed, -turn)

        val climbSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.rightStickButtonPressed) climbDirection = climbDirection.toggle()
        climb.set(climbDirection.sign() * climbSpeed)

        // Right trigger controls intake speed, A button to reverse
        val intakeSpeed = controller.rightTriggerAxis.pow(3.0)
        if (controller.aButtonPressed) intakeDirection = intakeDirection.toggle()
        intake.set(intakeDirection.sign() * intakeSpeed)

        shooterTop.set(-bottomSpeed)
        shooterBottom.set(-topSpeed)

        // Right x axis for turntable
        turntable.set(controller.rightX.deadzoneOne(0.05))

        // Left bumper to spin colon upwards, right for down
        if (controller.leftBumperPressed) {
            colon.set(-1.0)
        } else {
            colon.set(0.0)
        }
        
        if (controller.xButtonPressed) {
            leftSolenoid.set(when (leftSolenoid.get()) {
                DoubleSolenoid.Value.kOff, null -> DoubleSolenoid.Value.kForward
                DoubleSolenoid.Value.kForward -> DoubleSolenoid.Value.kReverse
                DoubleSolenoid.Value.kReverse -> DoubleSolenoid.Value.kReverse
            })
        }

        if (controller.bButtonPressed)
            centering = !centering
        
        println("Centering: $centering")

        ticks++
        
        if (centering) {
            val tv = limelightTable.getEntry("tv").getDouble(0.0)
            val tx = limelightTable.getEntry("tx").getDouble(0.0)

            println("Centering: tv: $tv, tx: $tx")

            if (tv == 1.0) {
                val turntableTurn = tv * (tx / 29.8 * 0.5).let { it.deadzoneOne(0.01) }
                turntable.set(turntableTurn)
            } else {
                // TODO: Naive radar sweep around 210 deg range if target not found 
                val turntableSpeed = (turntable.encoder.position / 60.0).clamp(-0.4, 0.4)
                println("Sweeping, speed: $turntableSpeed")
                // Turn clockwise for 50 ticks then counterclockwise for 50 ticks
                // if ((ticks % 100) > 50)
                //     turntable.set(0.1)
                // else
                //     turntable.set(-0.1)
            }
        }

        // topSpeed = topSpeedSlider!!.getDouble(0.0);
        // bottomSpeed = bottomSpeedSlider!!.getDouble(0.0);

        // // If save button pressed, write to CSV and reset to unpressed
        // if (saveButton!!.getBoolean(false)) {
        //     saveButton!!.forceSetBoolean(false)
        //     // writer.write("$topSpeed,$bottomSpeed")
        //     // writer.flush()
        // }
    }

    var solenoidMode = kForward;

    override fun testInit() {
    }
    override fun testPeriodic() {
        if (controller.leftBumperPressed) {
            solenoidMode = when (solenoidMode) {
                kForward -> kReverse
                else -> kForward
            }
            leftSolenoid.set(solenoidMode)
        } else if (controller.rightBumperPressed) {
            leftSolenoid.set(kOff)
        }
        if (controller.aButtonPressed) {
            pcmCompressor.enableDigital()
        } else if (controller.bButtonPressed) {
            pcmCompressor.disable()
        }
        println("Left solenoid: ${leftSolenoid.get().name}")
    }
}
