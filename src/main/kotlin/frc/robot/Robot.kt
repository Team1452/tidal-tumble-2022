package frc.robot

import kotlin.math.*

import frc.robot.main
import frc.robot.subsystems.DriveSubsystem
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
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.controller.PIDController
import java.io.IOException

import frc.robot.Drivetrain
import frc.robot.Constants

class Robot : TimedRobot(Constants.PERIOD) {
    var ticks = 0

    val controller = XboxController(0)
    val drive = DriveSubsystem(
        listOf(brushlessMotor(Constants.Real.LEFT_MOTOR_1), brushlessMotor(Constants.Real.LEFT_MOTOR_2)),
        listOf(brushlessMotor(Constants.Real.RIGHT_MOTOR_1), brushlessMotor(Constants.Real.RIGHT_MOTOR_2)),
        Pigeon2(Constants.Real.PIGEON)
    )

    val trajectoryJSON = "paths/top_blue.wpilib.json"
    var trajectory = Trajectory()

    override fun robotInit() {
        try {
            val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        } catch (err: IOException) {
            error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        }
    }

    override fun teleopPeriodic() {
        val speed = -controller.leftY.pow(3.0)
        val turn = controller.leftX.pow(3.0)
        drive.arcadeDrive(speed, turn)
    }

    override fun testInit() {
        val ramseteCommand = RamseteCommand(
            trajectory,
            drive::getPose,
            RamseteController(Constants.Test.Auto.RAMSETE_B, Constants.Test.Auto.RAMSETE_ZETA),
            SimpleMotorFeedforward(
                Constants.Test.Auto.KS_VOLTS,
                Constants.Test.Auto.KV_VOLT_SECONDS_PER_METER,
                Constants.Test.Auto.KA_VOLT_SECONDS_SQUARED_PER_METER
            ),
            Constants.DRIVE_KINEMATICS,
            drive::getWheelSpeeds,
            PIDController(Constants.Test.Auto.P_DRIVE_VEL, 0.0, 0.0),
            PIDController(Constants.Test.Auto.P_DRIVE_VEL, 0.0, 0.0),
            drive::tankDriveVolts,
            drive
        )

        drive.resetOdometry(trajectory.initialPose!!)

        ramseteCommand.andThen(object : Runnable { 
            override fun run() = drive.arcadeDrive(0.0, 0.0) 
        })
    }

    override fun testPeriodic() {
    }
    var sped = 0.0;
    var s = 1.0;
    var down = 0;
    override fun autonomousInit() {}
    override fun autonomousPeriodic() {

        drive.arcadeDrive(sped, 0.0)
        if(sped>=1.0){
            s = -1.0
        }
            sped = sped + (s*0.001)
    }

    override fun disabledInit() {}
    override fun disabledPeriodic() {}
}
