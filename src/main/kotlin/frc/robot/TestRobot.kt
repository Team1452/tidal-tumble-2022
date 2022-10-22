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
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.math.trajectory.TrajectoryUtil
import edu.wpi.first.math.trajectory.Trajectory
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import java.io.IOException
import com.ctre.phoenix.sensors.Pigeon2
import io.javalin.Javalin
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.PneumaticsModuleType

import frc.robot.Constants
import frc.robot.Drivetrain

// class TestRobot : TimedRobot(Constants.PERIOD) {
class TestRobot : TimedRobot(200.0/1000.0) {
    companion object {
        val LEFT_MOTOR = 6
        val RIGHT_MOTOR = 7
        val PIGEON = 10
    }

    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    // val drivetrain = Drivetrain(LEFT_MOTOR, RIGHT_MOTOR)

    var direction = Vec2(0.0, 0.0)
    var position = Vec2(0.0, 0.0)
    var lastDisplacement = 0.0

    val pigeon2 = Pigeon2(PIGEON)

    val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7)
    val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1)

    val trajectoryJSON = "resources/paths/topBlue.wpilib.json"
    var trajectory = Trajectory()
    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    var topSpeed:NetworkTableEntry? = null;
    var bottomSpeed:NetworkTableEntry? = null;
    override fun robotInit() {
        try {
            val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON)
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)
        } catch (err: IOException) {
            error("Unable to open trajectory: $trajectoryJSON, ${err.getStackTrace()}")
        }
        val tab = Shuffleboard.getTab("Test");
        
        topSpeed = tab.add("topSpeed", 1)
                   .getEntry();
        bottomSpeed =
                tab.add("bottomSpeed", 2)
                   .getEntry();
    }

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drivetrain.drive(speed, turn)
        shooterTop.set(topSpeed!!.getDouble(0.0));
        shooterBottom.set(bottomSpeed!!.getDouble(0.0));
    }
}