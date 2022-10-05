// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.networktables.NetworkTableInstance

import robot.Drivetrain
import kotlin.math.*
import com.ctre.phoenix.sensors.Pigeon2
import io.javalin.Javalin
import io.javalin.websocket.WsContext
import java.util.concurrent.ConcurrentHashMap

class Robot : TimedRobot() {
    companion object {
        val LEFT_MOTOR = 6
        val RIGHT_MOTOR = 7
        val PIGEON = 14
    }

    val limelightTable = NetworkTableInstance.getDefault().getTable("limelight")

    val controller = XboxController(0)
    val drivetrain = Drivetrain(LEFT_MOTOR, RIGHT_MOTOR)

    var direction = Vec2(0.0, 0.0)
    var position = Vec2(0.0, 0.0)
    var lastDisplacement = 0.0

    val pigeon2 = Pigeon2(PIGEON)

    val positionWSObservers: ConcurrentHashMap<String, WsContext> = ConcurrentHashMap<String, WsContext>()
    var app = Javalin.create().apply {
        ws("/position") { ws -> 
            ws.onConnect { ctx -> 
                positionWSObservers[ctx.sessionId] = ctx
                println("New websocket connected: $ctx")
            }
            ws.onClose { ctx -> 
                positionWSObservers.remove(ctx.sessionId)
                println("Websocket closed: $ctx")
            }
            ws.onMessage { ctx -> 
                ctx.session.remote.sendString("Echo: ${ctx.message()}")
            }
        }
    }.start(7070)

    override fun teleopInit() {}
    override fun teleopPeriodic() {
        val speed = controller.leftTriggerAxis.pow(3.0)
        val turn = controller.rightTriggerAxis.pow(3.0)
        drivetrain.drive(speed, turn)
    }

    override fun testInit() {}
    override fun testPeriodic() {
        // Positioning
        val yawRad = pigeon2.yaw * PI/180.0
        direction = Vec2(sin(yawRad), cos(yawRad))

        val displacement = (drivetrain.left.encoder.position + drivetrain.right.encoder.position)/2.0
        position += (displacement - lastDisplacement) * direction
        lastDisplacement = displacement

        positionWSObservers.values.forEach { it.session.remote.sendString("${position.x}, ${position.y}") }

        // Limelight
        val tv = limelightTable.getEntry("tv").getDouble(0.0)
        val tx = limelightTable.getEntry("tx").getDouble(0.0)

        val turn = tv * (tx / 29.8 * 0.5).apply { if (this > 0.05) this else 0.0 }
        drivetrain.drive(0.0, turn)
    }
}
