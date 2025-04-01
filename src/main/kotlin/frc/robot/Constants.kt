// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*

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

    object Vision {
        val reefTags = arrayOf(6,7,8,9,10,11)
    }
    object Elevator {
        val motorId2 = 52;
        val MAX_VEL = 1.5;
        val MAX_ACELERATION = 2.5;

        val MAX_HEIGHT = Meters.of(0.800) + Inches.of(1.0 - 0.125)//Meters.of(71.25 - 39.25)

        // TODO FIND THE RIGHT HEIGHTS val COLLISION_HEIGHT_LOW = Distance.ofBaseUnits(0.2, edu.wpi.first.units.Units.Meters)
        val COLLISION_HEIGHT_HIGH: Distance = Inches.of(53.0 - 39.25)
        val COLLISION_HEIGHT_LOW: Distance = Inches.of(48.0 - 39.25)

        const val motorId: Int = 51
        const val TOLERANCE = 0.03
        val GEAR_CIRCUMFERENCE =
            Inches.of(1.790 * PI)
                .`in`(edu.wpi.first.units.Units.Meters)
        val CONVERSION_FACTOR: Double = GEAR_CIRCUMFERENCE/9.0
    }

    object Arm {
        val LOW_LAST_SAFE_ANGLE = Rotation2d.fromDegrees(-245.0);
        val START_POSITION = -90.0;
        val MAX_ACELERATION = 3000.0;
        const val MAX_VEL = 4000.0;
        val SAFE_ANGLE = Rotation2d.fromDegrees(-150.394)
        const val motorId: Int = 53
        const val TOLERANCE = 2.0
        const val CONVERSION_FACTOR = 1.0/16.0 * 12.0/40.0 * 360.0;
        const val ABSOLUTE_ENCODER_CONVERSION_FACTOR = 12.0/40.0 * 360.0;
        // const val ABSOLUTE_ENCODER_CONVERSION_FACTOR = 1.0;

        // const val ABSOLUTE_ENCODER_OFFSET = -26.025 + 360.0;
        const val ABSOLUTE_ENCODER_OFFSET = 0.197 //0.439 // -0.241 + 1.0;

    }
    object Grappling {
        val MAX_ACELERATION = 250.0;
        const val MAX_VEL = 250.0;
        val SAFE_ANGLE = Rotation2d.fromDegrees(-90.0 - 32.385)
        const val motorId: Int = 9
        const val motor2Id: Int = 8
        const val TOLERANCE = 2.0
        const val CONVERSION_FACTOR = 1.0/16.0 * 12.0/40.0 * 360.0;
        const val ABSOLUTE_ENCODER_CONVERSION_FACTOR = 12.0/40.0 * 360.0;
        // const val ABSOLUTE_ENCODER_CONVERSION_FACTOR = 1.0;

        // const val ABSOLUTE_ENCODER_OFFSET = -26.025 + 360.0;

    }

    object Intake {
        const val MOTOR2 = 62;
        const val INTAKE: Double = -0.45
        const val INTAKE_ALGAE: Double = 11.0
        const val OUTPUT_ALGAE: Double = -9.0
        const val MOTOR = 61;
        const val OUTPUT_L1: Double = 0.3
        const val OUTPUT_L2NEW: Double = 0.6
        const val OUTPUT_L2: Double = 0.4
        const val OUTPUT_L3: Double = 0.4
        const val OUTPUT_L4: Double = 0.8
        const val L1OUTPUT: Double = 0.5
        const val STOP_CURRENT: Double = 40.0
    }

    const val ROBOT_MASS: Double = (148.0 - 20.3) * 0.453592 // 32lbs * kg per pound
    @JvmField
    val CHASSIS: Matter = Matter(Translation3d(0.0, 0.0, Units.inchesToMeters(8.0)), ROBOT_MASS)
    const val LOOP_TIME: Double = 0.13 //s, 20ms + 110ms sprk max velocity lag
//    val MAX_SPEED: Double = 3.0

    val ROBOT_WIDTH: Distance = Inches.of(3.0 * 2.0 + 27.0)

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
        const val DEADBAND: Double = 0.03
        const val LEFT_Y_DEADBAND: Double = 0.1
        const val RIGHT_X_DEADBAND: Double = 0.1
        const val TURN_CONSTANT: Double = 6.0
    }

    object Drivebase {
        val MAX_TURNING_SPEEDS = 4.5 * 0.9
        val MAX_SPEED = 4.0
        val ROTATION_PID_TELEOP = PIDConstants(2.4, 0.0, 0.5)
        //val LATERAL_PID_TELEOP = PIDConstants(2.5, 0.0, 0.0)
         val TRANSLATION_PID_TELEOP = PIDConstants(2.3, 0.0, 0.2)
        val REEF_TRANSLATION_PID_TELEOP = PIDConstants(2.0, 0.0, 0.008)

    }

    object Field {
        var LEFT_OFFSET = Inches.of(-5.25);
        var RIGHT_OFFSET = Inches.of(7.0);
//        val LEFT_OFFSET = Inches.of(-6.5);
//        val RIGHT_OFFSET = Inches.of(6.5);


    }

    object Poses {
        val L1 = Pose(
            Meters.of(0.0),
            Rotation2d.fromDegrees(-77.021),
            "l1"
        );

        val L2NEW = Pose(
            Meters.of(0.049) + Inches.of(2.0 - 0.125),
            Rotation2d.fromDegrees(-90.0),
            "l1new"
        );
//        );

//        val L2 = Pose(
//            Meters.of(0.0),
//            Rotation2d.fromDegrees(-77.021),
//            "l2"
//        );

        val CLIMB_POSE = Pose(
            Meters.of(0.420) - Inches.of(0.125),
            Rotation2d.fromDegrees(-359.801),
            "climb_pose"
        );

        val L2 = Pose(
            Meters.of(0.0),
            Rotation2d.fromDegrees(-219.6 - 7.0),
            "l2"
        );

        val L3 = Pose(
            Meters.of(0.225).plus(Inches.of(2.0 - 0.125)),
            Rotation2d.fromDegrees(-236.233),
            "l3"
        );

        val L4 = Pose(
            Constants.Elevator.MAX_HEIGHT,//Meters.of(0.8),
            // Rotation2d.fromDegrees(-241.0),
            Rotation2d.fromDegrees(-242.5 - -360.0/42.0/2.0),
            "l4"
        );

        val L4Back = Pose(
            Constants.Elevator.MAX_HEIGHT,//Meters.of(0.8),
            // Rotation2d.fromDegrees(-241.0),
            Rotation2d.fromDegrees(-250.0 - 15.0),
            "l4-back"
        );

        val Pickup = Pose(
            Meters.of(0.049) + Inches.of(3.0 - 0.125 ),
            Rotation2d.fromDegrees(-68.0),
            "pickup"
        );

        val Zero = Pose(
            Meters.of(0.0),
            Rotation2d.fromDegrees(-90.0),
            "zero"
        );

        val Barge = Pose(
            Constants.Elevator.MAX_HEIGHT,// - Inches.of(3.0),
            Rotation2d.fromDegrees(-290.00),
            "barge"
        );

        val Barge2 = Pose(
            Constants.Elevator.MAX_HEIGHT,// - Inches.of(3.0),
            Rotation2d.fromDegrees(-260.00),
            "barge"
        );
        
        val Processor = Pose(
            Meters.of(0.0),
            Rotation2d.fromDegrees(-65.0),
            "processor"
        );

        val L2Algae = Pose(
            Meters.of(0.0) + Inches.of(2.0 - 0.125),
            Rotation2d.fromDegrees(-131.00),
            "l2algae"
        );

        val L3Algae = Pose(
            Meters.of(0.153) + Inches.of(2.5) + Inches.of(2.0 - 0.125),
            Rotation2d.fromDegrees(-151.00),
            "l3algae"
        );
        val FullExtend = Pose(
            Elevator.MAX_HEIGHT,
            Constants.Arm.SAFE_ANGLE - Rotation2d.fromDegrees(10.0),
            "fullextend"
        );
    }
}
