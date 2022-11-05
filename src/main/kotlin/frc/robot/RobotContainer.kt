/*
package frc.robot

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.ctre.phoenix.sensors.Pigeon2
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.controller.PIDController
import java.io.IOException

import frc.robot.constants.*

class RobotContainer {
    val drive = DriveSubsystem(
        listOf(CANSparkMax(Constants.Test.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)),
        listOf(CANSparkMax(Constants.Test.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless)),
        Pigeon2(Test.PIGEON)
    )

    val controller = XboxController(0)

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()

    init {
        val button = JoystickButton(controller, Button.kRightBumper.value)
        
        button.whenPressed(() -> drive.maxOutput = 0.5)
        button.whenReleased(() -> drive.maxOutput = 1.0)

        drive.setDefaultCommand(
            RunCommand(
                { drive.arcadeDrive(-controller.leftY, controller.rightX) },
                drive
            )
        )

        try {
            val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        } catch (err: IOException) {
            error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        }
    }

    fun getAutonomousCommand() {
        val ramseteCommand = RamseteCommand(
            trajectory,
            drive::getPose,
            RamseteController(Constants.Auto.RAMSETE_B, Constants.Auto.RAMSETE_B),
            SimpleMotorFeedforward(
                Constants.KS_VOLTS,
                Constants.KV_VOLT_SECONDS_PER_METER,
                Constants.KA_VOLT_SECONDS_SQUARED_PER_METER
            ),
            Constants.DRIVE_KINEMATICS,
            drive::getWheelSpeeds,
            PIDController(Constants.P_DRIVE_VEL, 0.0, 0.0),
            PIDController(Constants.P_DRIVE_VEL, 0.0, 0.0),
            drive::tankDriveVolts,
            drive
        )

        drive.resetOdometry(trajectory.initialPose)

        ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
    }
}
p */