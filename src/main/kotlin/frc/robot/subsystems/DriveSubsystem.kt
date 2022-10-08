package frc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Pose2d
import frc.robot.getRotation2d

fun Pigeon2.getRotation2d(): Rotation2d = Rotation2d(Math.toRadians(this.yaw))

class SparkGroupEncoder(val motors: List<CANSparkMax>) {
    val position: Double
        get() = motors.fold(0.0) { acc, motor -> acc + motor.encoder.position } / motors.size
            * Constants.WHEEL_CIRCUMFERENCE
    
    var lastPosition = position
    var rate = 0.0

    fun periodic(msElapsed: Int) {
        val currentPosition = position
        rate = (currentPosition - lastPosition) / msElapsed.toDouble()
        lastPosition = currentPosition
    }

    fun reset() {
        motors.forEach { it.encoder.position = 0.0 }
        lastPosition = 0.0
    }
}

class DriveSubsystem(
    val left: List<CANSparkMax>,
    val right: List<CANSparkMax>,
    val pigeon: Pigeon2
) : SubsystemBase() {
    val odometry = DifferentialDriveOdometry(pigeon.getRotation2d())

    val leftEncoder = SparkGroupEncoder(left)
    val rightEncoder = SparkGroupEncoder(right)

    val lastYaw = pigeon.yaw
    var gyroRate = 0.0

    var maxOutput = 1.0

    init {
        right.forEach { it.inverted = true }
    }

    override fun periodic() {
        gyroRate = (pigeon.yaw - lastYaw) / Constants.COMMAND_SCHEDULER_PERIOD_SECONDS

        odometry.update(
            pigeon.getRotation2d(),
            leftEncoder.position,
            rightEncoder.position
        )
    }

    fun getPose(): Pose2d = odometry.poseMeters

    fun getWheelSpeeds(): DifferentialDriveWheelSpeeds =
        DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate) 

    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        odometry.resetPosition(pose, pigeon.getRotation2d())
    }

    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }

    fun arcadeDrive(forward: Double, rotation: Double) {
        val leftSpeed = (forward + rotation) * maxOutput
        val rightSpeed = (forward - rotation) * maxOutput

        left.forEach { it.set(leftSpeed) }
        right.forEach { it.set(rightSpeed) }
    }

    fun zeroHeading() {
        pigeon.setYaw(0.0, 0)
    }

    fun getHeading(): Double = pigeon.getRotation2d().getDegrees()

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        left.forEach { it.setVoltage(leftVolts) }
        right.forEach { it.setVoltage(rightVolts) }
    }
}