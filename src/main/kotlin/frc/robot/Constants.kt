package frc.robot

import kotlin.math.*
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics

public object Constants { 
    val WHEEL_CIRCUMFERENCE_INCHES = PI * 7.0.pow(2.0)

    val TRACK_WIDTH_METERS = 0.69
    val DRIVE_KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH_METERS)

    val PERIOD_MS = 20.0
    val PERIOD = PERIOD_MS / 1000.0

    val COMMAND_SCHEDULER_PERIOD_SECONDS = 20/1000

    object Auto {
    }

    object Test {
        val LEFT_MOTOR = 6
        val RIGHT_MOTOR = 7

        val PIGEON = 10

        object Auto {
            val KS_VOLTS = 0.22
            val KV_VOLT_SECONDS_PER_METER = 0.22
            val KA_VOLT_SECONDS_SQUARED_PER_METER = 0.22

            val P_DRIVE_VEL = 8.5

            val MAX_SPEED_METERS_PER_SECOND = 3.0
            val MAX_ACCELERATION_METERS_PER_SECOND = 1.0

            val RAMSETE_B = 2.0
            val RAMSETE_ZETA = 0.7
        }
    }

    object Real {
        // TODO: Solenoid ports
        val PCM = 0
        val LEFT_SOLENOID_1 = 4
        val LEFT_SOLENOID_2 = 5

        val RIGHT_SOLENOID_1 = 1
        val RIGHT_SOLENOID_2 = 0

        val CSV_PATH = "shooter_config.csv";
        val RIGHT_MOTOR_1 = 11
        val RIGHT_MOTOR_2 = 18

        val LEFT_MOTOR_1 = 15
        val LEFT_MOTOR_2 = 12

        // val PIGEON = 10
        val INTAKE = 20
        val COLON = 14
        val CLIMB = 17

        val TURNTABLE = 21
        val TURNTABLE_GEAR_RATIO = 14.0 / 1.066

        val SHOOTER_BOTTOM = 16 
        val SHOOTER_TOP = 13

        val SHOOTER_BOTTOM_GEAR_RATIO: Double = 2.0 
    }
}