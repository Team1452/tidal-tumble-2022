package frc.robot

import kotlin.math.*
import kotlin.io.path.Path

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.controller.PIDController
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.IOException
import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import io.javalin.Javalin
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.Command

import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

// class TestRobot : TimedRobot(Constants.PERIOD) {
class TestRobot : TimedRobot(20.0/1000.0) {
    companion object {
        val LEFT_MOTOR = 6
        val RIGHT_MOTOR = 7
        val PIGEON = 10
    }

    // val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    // val drivetrain = Drivetrain(LEFT_MOTOR, RIGHT_MOTOR)
    val drive = DriveSubsystem(
        listOf(brushlessMotor(Constants.Test.LEFT_MOTOR)),
        listOf(brushlessMotor(Constants.Test.RIGHT_MOTOR)),
        Pigeon2(Constants.Test.PIGEON)
    )

    var direction = Vec2(0.0, 0.0)
    var position = Vec2(0.0, 0.0)
    var lastDisplacement = 0.0

    val pigeon2 = Pigeon2(PIGEON)

    // val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7)
    // val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()

    override fun robotInit() {
        // try {
        //     val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        // } catch (err: IOException) {
        //     error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        // }
    }

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftY.pow(3.0)
        val turn = controller.leftX.pow(3.0)
        drive.arcadeDrive(speed, turn)
    }

    override fun testInit() {}
    override fun testPeriodic() {
        // try {
        //     val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        // } catch (err: IOException) {
        //     error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        // }

        // val ramseteCommand = RamseteCommand(
        //     trajectory,
        //     drive::getPose,
        //     RamseteController(Constants.Test.Auto.RAMSETE_B, Constants.Test.Auto.RAMSETE_ZETA),
        //     SimpleMotorFeedforward(
        //         Constants.Test.Auto.KS_VOLTS,
        //         Constants.Test.Auto.KV_VOLT_SECONDS_PER_METER,
        //         Constants.Test.Auto.KA_VOLT_SECONDS_SQUARED_PER_METER
        //     ),
        //     Constants.DRIVE_KINEMATICS,
        //     drive::getWheelSpeeds,
        //     PIDController(Constants.Test.Auto.P_DRIVE_VEL, 0.0, 0.0),
        //     PIDController(Constants.Test.Auto.P_DRIVE_VEL, 0.0, 0.0),
        //     drive::tankDriveVolts,
        //     drive
        // )

        // drive.resetOdometry(trajectory.initialPose!!)

        // TODO: Pass command properly
        // ramseteCommand.andThen(Command(() -> drive.tankDriveVolts(0.0, 0.0)))
    }
}