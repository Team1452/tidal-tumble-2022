package frc.robot

import kotlin.math.*

operator fun Double.times(o: Vec2) = o * this

class Vec2(val x: Double, val y: Double) {
    operator fun plus(o: Vec2): Vec2 = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2): Vec2 = Vec2(x - o.x, y - o.y)
    operator fun times(s: Double): Vec2 = Vec2(s * x, s * y)
    operator fun unaryMinus(): Vec2 = (-1.0) * this
    operator fun div(s: Double): Vec2 = this * (1.0/s)
    override operator fun equals(other: Any?): Boolean = when (other) {
        is Vec2 -> (this - other).norm() < 0.0001 // Inequality b/c weird floating point errors 
        else -> throw Exception("Tried to check equality of Vec2 with something other than a vector")
    }

    fun dot(o: Vec2): Double = x * o.x + y * o.y
    fun norm(): Double = sqrt(x.pow(2.0) + y.pow(2.0))
    fun hat(): Vec2 = this/norm()
    fun rotate(rad: Double): Vec2 = Vec2(cos(rad) * x - sin(rad) * y, sin(rad) * x + cos(rad) * y)
    fun rotateDeg(deg: Double): Vec2 = rotate(deg * PI/180.0)
}