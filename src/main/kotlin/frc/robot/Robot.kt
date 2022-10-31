package frc.robot

import kotlin.math.*

import frc.robot.main
import frc.robot.subsystems.DriveSubsystem
import io.javalin.Javalin
import io.javalin.websocket.WsContext
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.cameraserver.CameraServer
import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import java.util.concurrent.ConcurrentHashMap
import java.nio.ByteBuffer

import frc.robot.Drivetrain
import frc.robot.Constants
import org.opencv.core.Mat

abstract class Toggleable(private val _active: Boolean = false) {
    open fun toggleOn() {}
    open fun toggleOff() {}
    var active: Boolean = false
        set(v) = if (v) toggleOn() else toggleOff()
    init { active = _active }
    fun toggle() { active = !active }
}

class CompressorToggle(val compressor: Compressor) : Toggleable() {
    override fun toggleOn() = compressor.enableDigital()
    override fun toggleOff() = compressor.disable()
}

class IsForward : Toggleable() {
    val sign: Double
        get() = if (active) 1.0 else -1.0
}

class DirectionalMotor(val motor: CANSparkMax, val _forward: Boolean = true) : Toggleable() {
    val isForward = IsForward()
    var forward: Boolean = _forward
        set(forward) { isForward.active = forward }

    fun switchDirections() = isForward.toggle()
    fun set(speed: Double) = motor.set(isForward.sign * speed)
}

enum class MessageType(val index: Int) {
    FRAME_LIMELIGHT(0),
    FRAME_USB_CAMERA(1),
    SET_DRIVETRAIN(2),
    SET_CLIMB(3),
    SET_INTAKE(4),
    SET_COLON(5),
    SET_SHOOTER_TOP(6),
    SET_SHOOTER_BOTTOM(7),
    SET_COMPRESSOR(8),
    SET_SOLENOID(9)
}

class Parser(val raw: ByteArray) {
    var index = 0

    fun readType(): MessageType = MessageType.values()[raw[index++].toInt()]

    fun readFrame(): Mat {
        val width = raw[index++].toInt() shl 8 or raw[index++].toInt()
        val height = raw[index++].toInt() shl 8 or raw[index++].toInt()
        val buffer = ByteBuffer.wrap(raw, 5, width * height) // Slice of byte buffer is moved to Mat
        index += buffer.size
        return Mat(width, height, 4, buffer) // 4 channels for RGBA
    }

    fun readDouble(): Double {
        val result = Double.fromBits((0..8).map { raw[index + it+1].toLong() shl 64-(index + it+1)*8 }.fold(0L) { it, acc -> it or acc })
        index += 8
        return result
    }

    fun readBoolean(): Boolean {
        return raw[2] == 1.toByte()
    }
}

fun ByteArray.toByteBuffer(): ByteBuffer = ByteBuffer.wrap(this, 0, this.size)

object class Serializer {
    fun serializeCameraFrame(camera: MessageType, frame: Mat): ByteArray {
        val buffer = ByteArray(1 + frame.width() * frame.height() * 3)
        buffer[0] = camera.ordinal.toByte()
        var index = 0
        for (x in 0..frame.width()) {
            for (y in 0..frame.height()) {
                val pixel = ByteArray(3)
                frame.get(x, y, pixel)
                buffer[1 + index] = pixel[0]
                buffer[1 + index + 1] = pixel[1]
                buffer[1 + index + 2] = pixel[2]
                index += 3
            }
        }
        return buffer
    }

    fun serializeSet(type: MessageType, value: Double): ByteArray {
        val buffer = ByteArray(65)
        buffer[0] = type.ordinal.toByte()
        val bits = value.toBits()
        for (i in 1..65)
            buffer[i] = ((bits shr (64-i*8)) and 0xff).toByte()
        return buffer
    }

    fun serializeSet(type: MessageType, value: Boolean): ByteArray {
        return arrayOf(type.ordinal.toByte(), if (value) 1 else 0).toByteArray()
    }
}

class Robot : TimedRobot(Constants.PERIOD) {
    var ticks = 0

    val controller = XboxController(0)
    val drive = DriveSubsystem(
        listOf(Constants.Real.LEFT_MOTOR_1, Constants.Real.LEFT_MOTOR_2),
        listOf(Constants.Real.RIGHT_MOTOR_1, Constants.Real.RIGHT_MOTOR_2),
        Pigeon2(Constants.Real.PIGEON)
    )
    var speed = 0.0
    var turn = 0.0

    val intake = DirectionalMotor(brushlessMotor(Constants.Real.INTAKE))
    var intakeSpeed = 0.0

    val colon = brushlessMotor(Constants.Real.COLON)
    var colonSpeed = 0.0

    val climb = DirectionalMotor(brushlessMotor(Constants.Real.CLIMB))
    var climbSpeed = 0.0

    // Shooting
    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")
    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)
    var shooterTopSpeed = 0.0
    var shooterBottomSpeed = 0.0
    val turntable = brushlessMotor(Constants.Real.TURNTABLE)
    var turntableSpeed = 0.0
    var centering = false

    // Pneumatics
    val solenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Real.LEFT_SOLENOID_1, Constants.Real.LEFT_SOLENOID_2)
    var solenoidForward: Boolean? = null
    val compressor = CompressorToggle(Compressor(0, PneumaticsModuleType.CTREPCM))

    // Camera
    val cvSink = CameraServer.getVideo()
    val currentFrame = Mat()

    var connections = ConcurrentHashMap<String, WsContext>()
    var app = Javalin.create().apply {
        ws("/control") { ws -> 
            ws.onConnect { ctx -> 
                connections[ctx.sessionId] = ctx
                println("New websocket connected: ${ctx.sessionId}")
            }
            ws.onClose { ctx -> 
                connections.remove(ctx.sessionId)
                println("Websocket closed: ${ctx.sessionId}")
            }
            ws.onMessage { ctx -> 
                val msg = ctx.message().toByteArray()
                val parser = Parser(msg)
                when (parser.readType()) {
                    MessageType.FRAME_LIMELIGHT, MessageType.FRAME_USB_CAMERA -> {} // Only used for sending from server to client
                    MessageType.SET_DRIVETRAIN -> {
                        speed = parser.readDouble()
                        turn = parser.readDouble()
                    }
                    MessageType.SET_CLIMB -> climbSpeed = parser.readDouble()
                    MessageType.SET_INTAKE -> intakeSpeed = parser.readDouble()
                    MessageType.SET_COLON -> colonSpeed = parser.readDouble()
                    MessageType.SET_SHOOTER_TOP -> shooterTopSpeed = parser.readDouble()
                    MessageType.SET_SHOOTER_BOTTOM -> shooterBottomSpeed = parser.readDouble()
                    MessageType.SET_COMPRESSOR -> compressor.active = parser.readBoolean()
                    MessageType.SET_SOLENOID -> solenoidForward = parser.readBoolean()
                }
            }
        }
    }.start(7070)

    override fun robotInit() {
        compressor.active = false
    }
    override fun robotPeriodic() {
        ticks++
        cvSink.grabFrame(currentFrame)
        connections.values.forEach { it.send(Serializer.serializeCameraFrame(MessageType.FRAME_LIMELIGHT, currentFrame).toByteBuffer()) }
    }

    override fun teleopInit() {}

    override fun teleopPeriodic() {
        drive.arcadeDrive(speed, turn)

        climb.set(climbSpeed)

        intake.set(intakeSpeed)

        shooterTop.set(-shooterBottomSpeed)
        shooterBottom.set(-shooterTopSpeed)

        turntable.set(turntableSpeed)

        colon.set(colonSpeed)

        if (controller.xButtonPressed) {
            solenoid.set(when (solenoidForward) {
                null -> DoubleSolenoid.Value.kOff
                false -> DoubleSolenoid.Value.kReverse
                true -> DoubleSolenoid.Value.kForward
            })
        }

        if (controller.bButtonPressed)
            centering = !centering

        if (centering) {
            val tv = limelightTable.getEntry("tv").getDouble(0.0)
            val tx = limelightTable.getEntry("tx").getDouble(0.0)
            
            if (tv == 1.0) {
                val turntableTurn = tv * (tx / 29.8 * 0.5)
                turntable.set(turntableTurn)
            } else {
                turntable.set(0.0)
            }
        }
    }

    override fun testInit() {}
    override fun testPeriodic() {}

    override fun autonomousInit() {}
    override fun autonomousPeriodic() {}

    override fun disabledInit() {}
    override fun disabledPeriodic() {}
}
