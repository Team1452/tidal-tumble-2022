/*
 * This Kotlin source file was generated by the Gradle 'init' task.
 */
package robot

import kotlin.test.Test
import kotlin.test.assertNotNull

class Vec2Test {
    @Test
    fun add() {
        assertEquals(Vec2(1, 1) + Vec2(8, 9), Vec2(9, 10))
    }

    @Test
    fun dot() {
        assertEquals(Vec2(8, 2).dot(Vec2(3, 2)), 27)
    }

    // Etc.
}