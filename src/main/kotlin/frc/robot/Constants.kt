package frc.robot

import kotlin.math.*
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics

object Constants {
    public val WHEEL_CIRCUMFERENCE_INCHES = PI * 7.0.pow(2.0)

    val TRACK_WIDTH_METERS = 0.69
    val DRIVE_KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH_METERS)

    val COMMAND_SCHEDULER_PERIOD_SECONDS = 20/1000

    val KS_VOLTS = 0.22
    val KV_VOLT_SECONDS_PER_METER = 0.22
    val KA_VOLT_SECONDS_SQUARED_PER_METER = 0.22

    val P_DRIVE_VEL = 8.5

    public object Auto {
        val MAX_SPEED_METERS_PER_SECOND = 3.0
        val MAX_ACCELERATION_METERS_PER_SECOND = 1.0

        val RAMSETE_B = 2.0
        val RAMSETE_ZETA = 0.7
    }
        
    public object Test {
        val LEFT_MOTOR = 6
        val RIGHT_MOTOR = 7

        //val PIGEON = 10
    }

    public object Real {
        val CSV_PATH = "";
        val RIGHT_MOTOR_1 = 1
        val RIGHT_MOTOR_2 = 5

        val LEFT_MOTOR_1 = 15
        val LEFT_MOTOR_2 = 2

        val INTAKE = 14
        val CLIMB = 7

        val TURNTABLE = 8 // TODO

        // TODO
        val SHOOTER_BOTTOM = 6 
        val SHOOTER_TOP = 3

        val SHOOTER_BOTTOM_GEAR_RATIO = 2.5 / 1.5
    }
}