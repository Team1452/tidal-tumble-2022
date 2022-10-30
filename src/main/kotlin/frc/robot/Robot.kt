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
    val drivetrain = Drivetrain(
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

    var app = Javalin.create().apply {
        ws("/shooter") { ws -> 
            ws.onConnect { ctx -> 
                println("New websocket connected: $ctx")
            }
            ws.onClose { ctx -> 
                println("Websocket closed: $ctx")
            }
            ws.onMessage { ctx -> 
                try {
                    val _bottomSpeed = ctx.message().substringBefore(',').toDouble()
                    val _topSpeed = ctx.message().substringAfter(',').toDouble()
                    bottomSpeed = _bottomSpeed
                    topSpeed = _topSpeed
                    ctx.session.remote.sendString("CONFIG_UPDATED")
                    println("Received message from websocket: '${ctx.message()}', bottom speed: $bottomSpeed, top speed: $topSpeed")
                } catch (err: Exception) {
                    println("Malformed ws input: ${err.message}, error: ${err.message}") 
                }
            }
        }
    }.start(7070)

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
    var topSpeedSlider: NetworkTableEntry? = null
    var bottomSpeedSlider: NetworkTableEntry? = null
    var saveButton: NetworkTableEntry? = null

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()

    var speedConfigMode = false

    val pcmCompressor = Compressor(0, PneumaticsModuleType.CTREPCM)

    var ticks = 0;

    val pigeon = Pigeon2(Constants.Real.PIGEON)

    override fun robotInit() {
        pcmCompressor.disable()

        val tab = Shuffleboard.getTab("Main")
        topSpeedSlider = tab.add("Top Shooter", 0.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry()
        bottomSpeedSlider = tab.add("Bottom Shooter", 0.0).withWidget(BuiltInWidgets.kNumberSlider).getEntry()
    }

    var compressorOn = false
    var turnTableRotations = 0.0
    var acceleration = 0.0
    var speed = 0.0

    var direction = Vec2(0.0, 0.0)
    var position = Vec2(0.0, 0.0)
    var lastEncoderPosition = 0.0

    var targetSpeed = 0.0
    var currentSpeed = 0.0

    var ticksSinceInterp = 0.0

    var fineModeOn = false
    var scaling = 1.0

    fun smoothstep(a: Double, b: Double, t: Double): Double {
        return (2.0 * Math.pow(t, 3.0) - 3.0 * Math.pow(t, 2.0)).clamp(0.0, 1.0) * (b-a) + a
    }

    override fun teleopInit() {
        intake.set(1.0)
    }

    override fun teleopPeriodic() {
        turnTableRotations = turntable.encoder.position

        // val turn = controller.leftY.pow(3.0)
        // acceleration = controller.leftX.pow(3.0) * 0.05
        // if (turn < 0.05){
        //      if (acceleration.absoluteValue < 0.01) {
        //          speed -= speed/60.0
        //      } else {
        //          speed += acceleration
        //      }
        // } else {
        //     speed = controller.leftX.pow(3.0)
        // }

        val newTargetSpeed = scaling * controller.leftY.pow(3.0)

        if (Math.abs(targetSpeed - newTargetSpeed) > 0.1) {
            ticksSinceInterp = 0.0
        }

        targetSpeed = newTargetSpeed
        val speed = smoothstep(currentSpeed, targetSpeed, ticksSinceInterp / 30.0)

        ticksSinceInterp++

        if (Math.abs(targetSpeed - speed) < 0.01) {
            ticksSinceInterp = 0.0
            currentSpeed = speed
        }

        // 2x^3 - 3x^2

        val turn = scaling * controller.leftX.pow(3.0)

        drivetrain.drive(speed, turn)

        if (ticks > 100) {
            intake.set(0.0)
        }

        val yaw = Math.toRadians(pigeon.yaw)
        direction = Vec2(Math.acos(yaw), Math.asin(yaw))
        val encoderPosition = (drivetrain.left.encoder.position + drivetrain.right.encoder.position)/2
        val displacement = encoderPosition - lastEncoderPosition
        lastEncoderPosition = encoderPosition
        position = displacement * direction

        if (ticks % 100 == 0)
            println("Position: $position, direction: $direction, yaw: ${pigeon.yaw}, $yaw")
        
        val climbSpeed = scaling * controller.leftTriggerAxis.pow(3.0)
        if (controller.rightStickButtonPressed) climbDirection = climbDirection.toggle()
        climb.set(climbDirection.sign() * climbSpeed)

        // Right trigger controls intake speed, A button to reverse
        val intakeSpeed = scaling * controller.rightTriggerAxis.pow(3.0)
        if (controller.aButtonPressed) intakeDirection = intakeDirection.toggle()
        intake.set(intakeDirection.sign() * intakeSpeed)

        shooterTop.set(-bottomSpeed)
        shooterBottom.set(-topSpeed)

        // shooterTop.set(-topSpeedSlider!!.getDouble(0.0))
        // shooterBottom.set(-topSpeedSlider!!.getDouble(0.0))

        // Right x axis for turntable
        turntable.set(controller.rightX.deadzoneOne(0.05))

        if (controller.rightTriggerAxis > 0.05 && leftSolenoid.get() == kForward) {
            colon.set(-0.7)
        } else {
            colon.set(0.0)
        }
        if (controller.yButtonPressed) {
            compressorOn = !compressorOn
            if (compressorOn)
                pcmCompressor.enableDigital()
            else
                pcmCompressor.disable()
        }
        
        if (controller.xButtonPressed) {
            leftSolenoid.set(when (leftSolenoid.get()) {
                DoubleSolenoid.Value.kOff, null -> DoubleSolenoid.Value.kForward
                DoubleSolenoid.Value.kForward -> DoubleSolenoid.Value.kReverse
                DoubleSolenoid.Value.kReverse -> DoubleSolenoid.Value.kForward
            })
        }

        if (controller.rightBumperPressed) {
            fineModeOn = !fineModeOn
            scaling = if (fineModeOn) 0.25 else 1.0
        }

        if (controller.bButtonPressed)
            centering = !centering

        ticks++
        
        if (centering) {
            val tv = limelightTable.getEntry("tv").getDouble(0.0)
            val tx = limelightTable.getEntry("tx").getDouble(0.0)
            
            if (tv == 1.0) {
                val turntableTurn = tv * (tx / 29.8 * 0.5).let { it.deadzoneOne(0.01) }
                turntable.set(turntableTurn)
            } else {
                // TODO: Naive radar sweep around 210 deg range if target not found 
                val turntableSpeed = (turntable.encoder.position / 60.0).clamp(-0.4, 0.4)
                // Turn clockwise for 50 ticks then counterclockwise for 50 ticks
                if (Math.abs(turnTableRotations) > 0){
                    
                }
                if (Math.abs(turnTableRotations) < 0){
                    
                }
                if ((ticks % 100) > 50)
                    turntable.set(0.1)
                else
                    turntable.set(-0.1)
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

    override fun autonomousInit() {}
    override fun autonomousPeriodic() {
        if (ticks < 5000/Constants.PERIOD_MS)
            drivetrain.drive(0.0, -0.2)
        else
            drivetrain.drive(0.0, 0.0)

        ticks++
    }
}
