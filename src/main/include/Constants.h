// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Constants {
    namespace Test {
        namespace Drivetrain {
            constexpr int RIGHT_MOTOR = 6;
            constexpr int LEFT_MOTOR = 7;
        }
    }

    namespace Real {
        namespace Drivetrain {
            constexpr int RIGHT_MOTOR_1 = 1;
            constexpr int RIGHT_MOTOR_2 = 2;
            constexpr int LEFT_MOTOR_1 = 15;
            constexpr int LEFT_MOTOR_2 = 6;
        }

        constexpr int INTAKE = 14;
        constexpr int CLIMB = 7;
    }
}
