package frc.robot

import kotlin.math.*

operator fun Double.times(o: Vec2) = o * this

class Vec2(val x: Double, val y: Double) {
    operator fun plus(o: Vec2): Vec2 = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2): Vec2 = Vec2(x - o.x, y - o.y)
    operator fun times(s: Double): Vec2 = Vec2(s * x, s * y)
    operator fun unaryMinus(): Vec2 = (-1.0) * this
    operator fun div(s: Double): Vec2 = this * (1.0/s)
    override operator fun equals(v: Any?): Boolean = when (v) {
        is Vec2 -> (this - v).norm() < 0.0001 // Inequality b/c weird Double errors 
        else -> throw Exception("Tried to check equality of Vec2 with something other than a vector")
    }

    fun dot(o: Vec2): Vec2 = Vec2(x * o.x, y * o.y)
    fun norm(): Double = sqrt(x.pow(2.0) + y.pow(2.0))
    fun hat(): Vec2 = this/norm()
}