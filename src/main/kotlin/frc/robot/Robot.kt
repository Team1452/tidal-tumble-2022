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

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.*
import frc.robot.main
import frc.robot.Scheduler

class Robot : TimedRobot(Constants.PERIOD_MS) {
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

    val colon = brushlessMotor(Constants.Real.COLON)
    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    val turntable = brushlessMotor(Constants.Real.TURNTABLE)

    var centering = false
    val xButtonLastPressed = false
    var ratio: Double = Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO 
    val scheduler = Scheduler()
    val solenoid = Solenoid

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        scheduler.tick()

        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drivetrain.drive(speed, turn)
        val intakeSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.leftBumperPressed) intakeIsForward = !intakeIsForward
        intake.set(if (intakeIsForward) intakeSpeed else -intakeSpeed)

        if (controller.bButtonPressed) ratio++
        if (controller.xButtonPressed) ratio--
        println(ratio/2)
        //turntable.set(controller.rightX.let { deadzone(it, 0.01) })
        shooterTop.set((-controller.rightY.pow(3.0)) * (ratio / 2.0))
        shooterBottom.set((-controller.rightY.pow(3.0)) / (ratio / 2.0))

        if (controller.yButtonPressed) centering = !centering
        if (controller.rightBumperPressed) centering = !centering

        if (controller.leftTriggerAxis > 0)
            scheduler.scheduleIn(10) {
                colon.set(1.0)
            }
        
        if (centering) {
            val tv = limelightTable.getEntry("tv").getDouble(0.0)
            val tx = limelightTable.getEntry("tx").getDouble(0.0)

            if (tv == 1.0) {
                val turntableTurn = tv * (tx / 29.8 * 0.5).let { deadzone(it, 0.01) }
                turntable.set(turntableTurn)
            } else {
                // TODO: Naive radar sweep around 210 deg range if target not found 
                // turntable.set(-turntable.encoder.position / 30)
            }
        }
    }
    override fun testInit() {}
    override fun testPeriodic() { }

    fun deadzone(v: Double, threshold: Double): Double = sign(v) * max(0.0, abs(v) - threshold)
}