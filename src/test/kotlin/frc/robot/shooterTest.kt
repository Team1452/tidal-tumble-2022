/*
package robot

package frc.robot

import kotlin.math.*

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.networktables.NetworkTableInstance
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap
import com.ctre.phoenix.sensors.Pigeon2
import com.revrobotics.CANSparkMax
import io.javalin.Javalin
import com.opencsv.*

import frc.robot.Drivetrain
import frc.robot.Constants
import kotlin.test.Test
import kotlin.test.assertEquals
import frc.robot.Vec2
import frc.robot.brushlessMotor
import java.util.Scanner
import java.io.FileWriter

class shooterTest { 
    val s =  Scanner(System.`in`);  // Create a Scanner object
    var top:CANSparkMax;
    var bottom:CANSparkMax;
    constructor(top: CANSparkMax, bottom: CANSparkMax) {
        this.top = top;
        this.bottom = bottom;
    }
    //adds data: [distance(meters), motorPower] to a csv file
    fun test(){
        print("distance: ")
        var distance = s.nextDouble();
        println("shooter speed: ");   
        val rI = s.nextLine().trim()
        val speed: Double = rI.toDouble();
        when(rI){
            "Y" -> log(distance, speed)
            else-> {
                println("shooting with speed: $speed \n at distance: $distance")
                top.set(speed)
                bottom.set(speed / Constants.Real.SHOOTER_BOTTOM_GEAR_RATIO)

            }
        }
    }

    var writer:CSVWriter = CSVWriter(FileWriter(Constants.Real.CSV_PATH, true); 
    private fun log(dist: Double, shooterSpeed: Double?): Array<String> {
        val arr:Array<String> = arrayOf(dist.toString(), shooterSpeed.toString())
        writer.writeNext(arr, false);
        writer.close();
        return arr;
    }


}


class Robot : TimedRobot() {
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

    val shooterTop = brushlessMotor(Constants.Real.SHOOTER_TOP)
    val shooterBottom = brushlessMotor(Constants.Real.SHOOTER_BOTTOM)

    val turntable = brushlessMotor(Constants.Real.TURNTABLE)

    enum StickMode {
        TURNTABLE, SHOOTER
    }

    var rightMode = StickMode.TURNTABLE
    val xButtonLastPressed = false

    val shooterTest = shooterTest(shooterTop, shooterBottom)

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftX.pow(3.0)
        val turn = controller.leftY.pow(3.0)
        drivetrain.drive(speed, turn)
        if(controller.yButtonPressed){
           shooterTest.test()
        }


        val intakeSpeed = controller.leftTriggerAxis.pow(3.0)
        if (controller.leftBumperPressed) intakeIsForward = !intakeIsForward
        intake.set(if (intakeIsForward) intakeSpeed else -intakeSpeed)

    }

    override fun testInit() {}
    override fun testPeriodic() {
   }
 */