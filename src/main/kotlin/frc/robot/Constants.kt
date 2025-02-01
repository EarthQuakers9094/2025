// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units

import edu.wpi.first.units.measure.Distance
import swervelib.math.Matter
import kotlin.math.PI

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    object Elevator {
        val MAX_HEIGHT = Distance.ofBaseUnits(71.25 - 39.25, edu.wpi.first.units.Units.Inches)

        // TODO FIND THE RIGHT HEIGHTS val COLLISION_HEIGHT_LOW = Distance.ofBaseUnits(0.2, edu.wpi.first.units.Units.Meters)
        val COLLISION_HEIGHT_HIGH: Distance = Distance.ofBaseUnits(53.0-39.25, edu.wpi.first.units.Units.Inches)
        val COLLISION_HEIGHT_LOW: Distance = Distance.ofBaseUnits(48.0-39.25, edu.wpi.first.units.Units.Meters)

        const val motorId: Int = -1
        const val TOLERANCE = 0.01
        private val GEAR_CIRCUMFERENCE =
            Distance.ofBaseUnits(1.790 * PI, edu.wpi.first.units.Units.Inch)
                .`in`(edu.wpi.first.units.Units.Meters)
        val CONVERSION_FACTOR = GEAR_CIRCUMFERENCE/9.0
    }

    object Arm {
        val SAFE_ANGLE = Rotation2d.fromDegrees(-90.0 - 32.385)
        const val motorId: Int = -1
        const val TOLERANCE = 2.0


    }

    object Intake {
        const val INTAKE: Double = -0.5
        const val MOTOR = -1;
        const val OUTPUT: Double = 0.5
    }

    const val ROBOT_MASS: Double = (148 - 20.3) * 0.453592 // 32lbs * kg per pound
    @JvmField
    val CHASSIS: Matter = Matter(Translation3d(0.0, 0.0, Units.inchesToMeters(8.0)), ROBOT_MASS)
    const val LOOP_TIME: Double = 0.13 //s, 20ms + 110ms sprk max velocity lag
    val MAX_SPEED: Double = Units.feetToMeters(14.5)

    // Maximum speed of the robot in meters per second, used to limit acceleration.
    //  public static final class AutonConstants
    //  {
    //
    //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
    //  }
    object DrivebaseConstants {
        // Hold time on motor brakes when disabled
        const val WHEEL_LOCK_TIME: Double = 10.0 // seconds
    }

    object OperatorConstants {
        const val LEFT_X_DEADBAND = 0.05

        // Joystick Deadband
        const val DEADBAND: Double = 0.1
        const val LEFT_Y_DEADBAND: Double = 0.1
        const val RIGHT_X_DEADBAND: Double = 0.1
        const val TURN_CONSTANT: Double = 6.0
    }

    object Drivebase {
        val MAX_TURNING_SPEEDS = 9.0
        val MAX_SPEED = 3.9624
        val ROTATION_PID_TELEOP = PIDConstants(0.4, 0.0, 0.0)
    }
    
    object Poses {
        val L1 = Pose(
            Distance.ofBaseUnits(0.0, edu.wpi.first.units.Units.Meters),
            Rotation2d.fromDegrees(-210.0),
        );

        val L2 = Pose(
            Distance.ofBaseUnits(0.0, edu.wpi.first.units.Units.Meters),
            Rotation2d.fromDegrees(-210.0),
        );

        val L3 = Pose(
            Distance.ofBaseUnits(0.0, edu.wpi.first.units.Units.Meters),
            Rotation2d.fromDegrees(-210.0),
        );

        val L4 = Pose(
            Distance.ofBaseUnits(0.0, edu.wpi.first.units.Units.Meters),
            Rotation2d.fromDegrees(-210.0),
        );

        val Pickup = Pose(
            Distance.ofBaseUnits(0.0, edu.wpi.first.units.Units.Meters),
            Rotation2d.fromDegrees(-45.0),
        );
    }
}
