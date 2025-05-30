// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import bindings.Constant
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*

import edu.wpi.first.units.measure.Distance
import swervelib.math.Matter
import kotlin.math.PI
import edu.wpi.first.math.trajectory.TrapezoidProfile


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared g`bally (i.e. public static). Do
 * not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

class PIDConstants(public var p: Double, public var i: Double, public var d: Double)

class Constants {
    object Vision {
        val reefTags = arrayOf(6,7,8,9,10,11) // not in gui because array and don't support that yet
    }

    class Elevator {
        var motorId2 = 0;
        var MAX_VEL = 0.0;
        var MAX_ACCELERATION = 0.0;
        var MAX_HEIGHT = 0.0;
        var COLLISION_HEIGHT_HIGH = 0.0;
        var COLLISION_HEIGHT_LOW = 0.0;
        var motorId = 0;
        var TOLERANCE = 0.0
        var gear_cicumference = 0.0;
        var conversion_factor = 0.0;
    }

    class DriveBaseConstants {
        var WHEEL_LOCK_TIME = 0.0;
    }

    var driveBaseConstants = DriveBaseConstants();


    var elevator = Elevator();

    class Arm {
        var LOW_LAST_SAFE_ANGLE = 0.0;
        var START_POSITION = 0.0;
        var MAX_ACELERATION = Constant(0.0);
        var MAX_VEL = Constant(0.0);
        var SAFE_ANGLE = 0.0;
        var motorId: Int = 0;
        var TOLERANCE = 0.0
        var CONVERSION_FACTOR = 0.0;
        var ABSOLUTE_ENCODER_CONVERSION_FACTOR = 0.0;
        var ABSOLUTE_ENCODER_OFFSET = 0.0;

        fun profile(): TrapezoidProfile {
            val cons = TrapezoidProfile.Constraints(MAX_VEL.getValue(), MAX_ACELERATION.getValue();

            val profile = TrapezoidProfile(cons);

            MAX_VEL.addListener {
                cons.maxVelocity = it;
            }

            MAX_ACELERATION.addListener {
                cons.maxAcceleration = it
            }

            return profile
        }
    }

    var arm = Arm();

    class Grappling {
        var MAX_ACELERATION = 0.0;
        var MAX_VEL = 0.0;
        var SAFE_ANGLE = 0.0;
        var motorId = 0
        var motor2Id= 0
        var TOLERANCE = 0.0
        var CONVERSION_FACTOR = 0.0;
        var ABSOLUTE_ENCODER_CONVERSION_FACTOR = 0.0;
    }

    var grappling = Grappling();

    class Intake {
        var MOTOR2 = 0;
        var INTAKE: Double = 0.0
        var INTAKE_ALGAE: Double = 0.0
        var OUTPUT_ALGAE: Double = 0.0
        var MOTOR = 0;
        var OUTPUT_L1: Double = 0.0
        var OUTPUT_L2NEW: Double = 0.0
        var OUTPUT_L2: Double = 0.0
        var OUTPUT_L3: Double = 0.0
        var OUTPUT_L4: Double = 0.0
        var L1OUTPUT: Double = 0.0
        var STOP_CURRENT: Double = 0.0
    }

    var intake = Intake();

    var ROBOT_MASS = 0.0;
    var loop_time = 0.0;
    var max_speed = 0.0;
    var width = 0.0;

    class OperatorConstants {
        var LEFT_X_DEADBAND = 0.05

        // Joystick Deadband
        var DEADBAND: Double = 0.03
        var LEFT_Y_DEADBAND: Double = 0.1
        var RIGHT_X_DEADBAND: Double = 0.1
    }

    var operatorConstants = OperatorConstants();

    class Drivebase {
        var MAX_TURNING_SPEEDS = 0.0
        var MAX_SPEED = 0.0
        var ROTATION_PID_TELEOP = PIDConstants(0.0, 0.0, 0.0)
        var TRANSLATION_PID_TELEOP = PIDConstants(0.0, 0.0, 0.0)
        var REEF_TRANSLATION_PID_TELEOP = PIDConstants(0.0, 0.0, 0.0)
    }

    var drivebase = Drivebase();

    class Field {
        var LEFT_OFFSET = 0.0;
        var RIGHT_OFFSET = 0.0;
    }

    var field = Field();

    class Poses {
        var L1 = Pose(
            0.0,
            0.0,
            ""
        );

        var L2NEW = Pose(
            0.0,
            0.0,
            ""
        );

        var CLIMB_POSE = Pose(
            0.0,
            0.0,
            ""
        );

        var L2 = Pose(
            0.0,
            0.0,
            ""
        );

        var L3 = Pose(
            0.0,
            0.0,
            ""
        );

        var L4 = Pose(
            0.0,
            0.0,
            ""
        );

        var L4Back = Pose(
            0.0,//Meters.of(0.8),
            0.0,
            ""
        );

        var Pickup = Pose(
            0.0,
            0.0,
            ""
        );

        var Zero = Pose(
            0.0,
            0.0,
            ""
        );

        var Barge = Pose(
            0.0,
//            Constants.Elevator.MAX_HEIGHT - Meters.of(0.66666),// - Inches.of(3.0),
            0.0,
            ""
        );

        var BargePrep = Pose(
            0.0,
//            Constants.Elevator.MAX_HEIGHT - Meters.of(0.66666),// - Inches.of(3.0),
            0.0,
            ""
        );

        var Barge2 = Pose(
            0.0,// - Inches.of(3.0),
            0.0,
            ""
        );

        var Processor = Pose(
            0.0,
            0.0,
            ""
        );

        var L2Algae = Pose(
            0.0,
            0.0,
            ""
        );

        var L3Algae = Pose(
            0.0,
            0.0,
            ""
        );
        var FullExtend = Pose(
            0.0,
            0.0,
            ""
        );
    }

    var poses = Poses();
}