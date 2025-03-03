// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import com.reduxrobotics.sensors.canandmag.CanandmagDetails
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.*
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.commands.LaunchAlgaeCommand
import frc.robot.commands.ScoreAlgae
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.subsystems.GrapplingSubsystem
import frc.robot.utils.Config
import swervelib.SwerveInputStream
import java.io.File
import java.util.*
import java.util.stream.Stream
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.pow
import org.photonvision.PhotonCamera
import CameraAlignInfo
import VisionSubsystem
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    val driverLeftStick: CommandJoystick = CommandJoystick(0)
    val driverRightStick: CommandJoystick = CommandJoystick(1)
    val operatorXbox: CommandXboxController = CommandXboxController(2)

    var frame = 0

    var lastScaleFactor = 0.0;

    var lastUpdatedScaleFactor = -1;

     fun getScaleFactor(): Double {
        if (lastUpdatedScaleFactor == frame) {
            return lastScaleFactor
        }
        val pov = driverLeftStick.getHID().getPOV()
        // SmartDashboard.putNumber("squared input magnitude", driverLeftStick.getX().pow(2.0) + driverLeftStick.getY().pow(2.0))
        // val angle = atan2(driverLeftStick.getY(), driverLeftStick.getX()) % (Math.PI/2.0);
        val factor = /*sin((angle - Math.PI/4.0).absoluteValue + Math.PI/4.0) * */
        if (pov == 0) {
            (2.0/3.0).pow((1/3))
        } else if (pov == 180) {
            (1.0/3.0).pow((1/3))
        } else {
            1.0
        }

        lastScaleFactor = factor
        lastUpdatedScaleFactor = frame

        SmartDashboard.putNumber("driving scale factor", factor)
        return factor
    }

    // The robot's subsystems and commands are defined here...
    val drivebase = SwerveSubsystem(
        File(
            Filesystem.getDeployDirectory(),
            Config("testswerve/neo","swerve/neo").config
        )
    )

    private val subsystemsEnable = Config(false, true).config && RobotBase.isReal();

    private val armSubsystem = ArmSubsystem(
        if (subsystemsEnable) {
            ArmSubsystem.ArmNeoIO(Constants.Arm.motorId)
        } else {
            ArmSubsystem.ArmSimIO()
        })

    private val grappleSubsystem = GrapplingSubsystem(
        if (subsystemsEnable) {
            GrapplingSubsystem.GrapplingNeoIO(Constants.Grappling.motorId)
        } else {
            GrapplingSubsystem.GrapplingSimIO()
        })

    private val elevatorSubsystem = ElevatorSubsystem(
        if (subsystemsEnable) {
            ElevatorSubsystem.ElevatorNeoIO(Constants.Elevator.motorId, Constants.Elevator.motorId2)
        } else {
            ElevatorSubsystem.ElevatorSimIO()
        })

    private val intakeSubsystem = IntakeSubsystem(
        if (subsystemsEnable) {
            IntakeSubsystem.IntakeNeoIO(Constants.Intake.MOTOR, Constants.Intake.MOTOR2)
        } else {
            IntakeSubsystem.IntakeSym()
        })

    
    private val visionSubsystem = VisionSubsystem(
            if (subsystemsEnable) {
                VisionSubsystem.VisionRealIO(
                    PhotonCamera("ATFrontRight"), 
                    Inches.of(8.125),
                    PhotonCamera("ATFrontLeft"),
                    Inches.of(-8.0),
                    // PhotonCamera("ATBack"),
                    // Inches.of(0.0),
                )
                    
            } else {
                VisionSubsystem.VisionSimIO()
            })

    private val isCompetition = false;

    private val autoChooser: SendableChooser<Command>; 

    // /**
    //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    //  */
    var driveAngularVelocity: SwerveInputStream = SwerveInputStream.of(
        drivebase.swerveDrive,
        { driverLeftStick.getY() * getScaleFactor()},
        { driverLeftStick.getX() * getScaleFactor()})
        .withControllerRotationAxis { driverRightStick.getX() * -0.7 }
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true)

    // /**
    //  * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
    //  */
    // var driveDirectAngle: SwerveInputStream = driveAngularVelocity.copy().withControllerHeadingAxis(
    //     { driverXbox.rightX * getScaleFactor()},
    //     { driverXbox.rightY * getScaleFactor()})
    //     .headingWhile(true)


    // // Applies deadbands and inverts controls because joysticks
    // // are back-right positive while robot
    // // controls are front-left positive
    // // left stick controls translation
    // // right stick controls the desired angle NOT angular rotation
    // var driveFieldOrientedDirectAngle: Command = drivebase.driveFieldOriented(driveDirectAngle)

    // // Applies deadbands and inverts controls because joysticks
    // // are back-right positive while robot
    // // controls are front-left positive
    // // left stick controls translation
    // // right stick controls the angular velocity of the robot
    var driveFieldOrientedAnglularVelocity: Command = drivebase.driveFieldOriented(driveAngularVelocity)

    // var driveSetpointGen: Command = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle)



    // var driveAngularVelocitySim: SwerveInputStream = SwerveInputStream.of(
    //     drivebase.swerveDrive,
    //     { -driverXbox.leftY * getScaleFactor()},
    //     { -driverXbox.leftX * getScaleFactor()})
    //     .withControllerRotationAxis { driverXbox.getRawAxis(2) }
    //     .deadband(OperatorConstants.DEADBAND)
    //     .scaleTranslation(0.8)
    //     .allianceRelativeControl(true)

    // // Derive the heading axis with math!
    // var driveDirectAngleSim: SwerveInputStream = driveAngularVelocitySim.copy()
    //     .withControllerHeadingAxis(
    //         {
    //             sin(
    //                 driverXbox.getRawAxis(
    //                     2
    //                 ) * Math.PI
    //             ) * (Math.PI * 2)
    //         },
    //         {
    //             cos(
    //                 driverXbox.getRawAxis(
    //                     2
    //                 ) * Math.PI
    //             ) *
    //                     (Math.PI * 2)
    //         })
    //     .headingWhile(true)

    // var driveFieldOrientedDirectAngleSim: Command = drivebase.driveFieldOriented(driveDirectAngleSim)

    // var driveSetpointGenSim: Command = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim)


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the trigger bindings
        configureBindings()
        DriverStation.silenceJoystickConnectionWarning(true)
        NamedCommands.registerCommand("launch_coral", launch_coral(intakeSubsystem,armSubsystem))
        NamedCommands.registerCommand("devour_coral", DevourCoralCommand(intakeSubsystem, true))
        NamedCommands.registerCommand("l1", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L1))
        NamedCommands.registerCommand("l2", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L2))
        NamedCommands.registerCommand("l3", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L3))
        NamedCommands.registerCommand("zero_pose", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.Zero))

        NamedCommands.registerCommand("pickup_pose", BackupCommand(drivebase).andThen(gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.Pickup)))
        NamedCommands.registerCommand("l4", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L4))

        NamedCommands.registerCommand("align_first", alignReefSelect(drivebase, visionSubsystem, "align_first_left"));
        NamedCommands.registerCommand("align_second", alignReefSelect(drivebase, visionSubsystem, "align_second_left"));
        NamedCommands.registerCommand("align_third", alignReefSelect(drivebase, visionSubsystem, "align_third_left"));
        NamedCommands.registerCommand("align_fourth", alignReefSelect(drivebase, visionSubsystem, "align_fourth_left"));

        // for (tag in Constants.Vision.reefTags) {

        // NamedCommands.registerCommand("align_right_${tag}", AlignReef(drivebase, visionSubsystem, Inches.of(-6.5 - 0.5) , {tag}, true))
        // NamedCommands.registerCommand("align_left_${tag}", AlignReef(drivebase, visionSubsystem, Inches.of(6.5 - 0.5) , {tag}, true))
        // }
        NamedCommands.registerCommand("hello there this is me", AlignReef(drivebase, visionSubsystem, Inches.of(-6.5 - 0.5) , {10}, true))
        NamedCommands.registerCommand("align_left_${10}", AlignReef(drivebase, visionSubsystem, Inches.of(6.5 - 0.5) , {10}, true))

        drivebase.setupPathPlanner()

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            { stream: Stream<PathPlannerAuto> ->
                if (isCompetition) {
                    stream.filter({auto:PathPlannerAuto -> auto.getName().startsWith("comp")})
                } else {
                    stream
                } 
            });

        SmartDashboard.putData("Auto Chooser", autoChooser)

    }

    private fun getSelectedTag(): Int {
        if (driverLeftStick.getHID().getPOV() == 0) {
            return 10
        }
        if (driverLeftStick.getHID().getPOV() == 180) {
            return 7
        }
        if (driverLeftStick.getHID().getRawButton(5)) {
            return 11
        }
        if (driverLeftStick.getHID().getRawButton(6)) {
            return 9
        }
        if (driverLeftStick.getHID().getRawButton(3)) {
            return 6
        }
        if (driverLeftStick.getHID().getRawButton(4)) {
            return 8
        }
        return -1
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger.Trigger] constructor with an arbitrary predicate, or via the
     * named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for
     * [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        // (Condition) ? Return-On-True : Return-on-False
        // drivebase.defaultCommand =
        //     if (!RobotBase.isSimulation()) driveFieldOrientedDirectAngle else driveFieldOrientedDirectAngleSim

        if (RobotBase.isSimulation()) {
            // driverXbox.start().onTrue(Commands.runOnce({
            //     drivebase.resetOdometry(
            //         Pose2d(
            //             3.0,
            //             3.0,
            //             Rotation2d()
            //         )
            //     )
            // }))
        }
        if (DriverStation.isTest()) {
            //drivebase.defaultCommand = driveFieldOrientedAnglularVelocity // Overrides drive command above!

            // driverXbox.b().whileTrue(drivebase.sysIdAngleMotorCommand())
            // driverXbox.x().whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2))
            // driverXbox.start().onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            // driverXbox.back().whileTrue(drivebase.centerModulesCommand())
            // driverXbox.leftBumper().onTrue(Commands.none())
            // driverXbox.rightBumper().onTrue(Commands.none())
        } else {
            driverRightStick.button(11).onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            driverRightStick.button(10).whileTrue(MoveGrappleCommand(grappleSubsystem, -0.5))
            driverRightStick.button(8).whileTrue(MoveGrappleCommand(grappleSubsystem, 0.5))
            driverRightStick.button(12).whileTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.CLIMB_POSE))

            //driverXbox.x().onTrue(Commands.runOnce({ drivebase.addFakeVisionReading() }))
            // driverXbox.b().whileTrue(T
            //     drivebase.driveToPose(
            //         Pose2d(Translation2d(4.0, 4.0), Rotation2d.fromDegrees(0.0))
            //     )
            // )
            //driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2.0))
            // driverXbox.start().whileTrue(Commands.none())
            // driverXbox.back().whileTrue(Commands.none())
            //driverXbox.start().whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            driverRightStick.button(1).whileTrue(AlignReef(drivebase, visionSubsystem, Constants.Field.RIGHT_OFFSET
             , {getSelectedTag()},false
            ))
            driverLeftStick.button(1).whileTrue(AlignReef(drivebase, visionSubsystem, Constants.Field.LEFT_OFFSET,  {getSelectedTag()}, false))
            driverRightStick.button(2).whileTrue(AlignReef(drivebase, visionSubsystem, Inches.of(0.0) , {getSelectedTag()}, false))
            driverLeftStick.button(2).whileTrue(ParallelCommandGroup(
                gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Pickup),
                AlignPickup(drivebase, visionSubsystem, Inches.of(0.0))
            ))

            driverRightStick.button(3).onTrue(InstantCommand({
                setMotorBrake(true)
            }))
            driverRightStick.button(3).onFalse(InstantCommand({
                setMotorBrake(false)
            }))

            // operatorXbox.y().whileTrue(MoveArmCommand(armSubsystem,2.0))
            // operatorXbox.a().whileTrue(MoveArmCommand(armSubsystem,-2.0))

            // operatorXbox.b().whileTrue(InstantCommand({elevatorSubsystem.setSetpoint(elevatorSubsystem.getHeight().plus(Meters.of(0.05)))}))
            // operatorXbox.x().whileTrue(InstantCommand({elevatorSubsystem.setSetpoint(elevatorSubsystem.getHeight().minus(Meters.of(0.05)))}))
            operatorXbox.start().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L2Algae))
            operatorXbox.back().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L3Algae))
           operatorXbox.y().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L4))
           operatorXbox.b().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L3))
           operatorXbox.a().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L2))
           operatorXbox.x().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L1))
           operatorXbox.povDown().onTrue(
            /*Commands.select(
                mapOf(
                    false to ParallelCommandGroup(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Zero), BackupCommand(drivebase)),
                    */gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Zero)/*
                ),  
                {
                    armSubsystem.getPose() == Constants.Poses.Barge.pose
                }
                )*/
            )
           operatorXbox.povLeft().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Processor))
           //operatorXbox.povLeft().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend))

           operatorXbox.povRight().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Pickup))

           operatorXbox.povUp().onTrue(ScoreAlgae(intakeSubsystem, armSubsystem, elevatorSubsystem))

           operatorXbox.leftTrigger(0.1).onTrue(launch_coral(intakeSubsystem,armSubsystem))
           operatorXbox.rightTrigger(0.1)./*onTrue*/whileTrue(DevourCoralCommand(intakeSubsystem,false))
           operatorXbox.rightBumper()./*onTrue*/whileTrue(DevourAlgaeCommand(intakeSubsystem))

           operatorXbox.leftBumper().onTrue(LaunchAlgaeCommand(intakeSubsystem))

        //    operatorXbox.povLeft().onTrue(FlickAlgae(armSubsystem))

//
//            operatorXbox.rightBumper().whileTrue(LaunchCoralCommand(intakeSubsystem))
//
        }

        // fun applyTeam(speed: Double):Double {
        //     if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Blue)) {
        //         return -speed;
        //     } else {
        //         return speed;
        //     }
        // }

//        fun upMax(speed: Double):Double {
//            val sign = speed.sign;
//            return sign * Math.min(Math.abs(speed) - 0.1,1.0);
//        }

        // val leftY =
        //         {
                    
        //             -MathUtil.applyDeadband(
        //                 applyTeam(driverLeftStick.getY()),
        //                 Constants.OperatorConstants.LEFT_Y_DEADBAND
        //             )
        //         }


        // val leftX: () -> Double =
        //         {
        //             -MathUtil.applyDeadband(
        //                 applyTeam(driverLeftStick.getX()),
        //                 Constants.OperatorConstants.LEFT_X_DEADBAND
        //             )
                    
        //         };

        // val omega = {
        //     MathUtil.applyDeadband(
        //         driverRightStick.getX(),
        //         Constants.OperatorConstants.LEFT_X_DEADBAND
        //     )
        // }

        // val driveMode = { true }

        // val simClosedFieldRel =
        //     TeleopDrive(
        //         drivebase,
        //         leftY,
        //         leftX,
        //         omega,
        //         driveMode,
        //     )

        drivebase.defaultCommand = driveFieldOrientedAnglularVelocity;

        // drivebase.defaultCommand = simClosedFieldRel
    }

    val autonomousCommand: Command
        get() = autoChooser.selected

    fun setDriveMode() {
        configureBindings()
    }

    fun setMotorBrake(brake: Boolean) {
        drivebase.setMotorBrake(brake)
    }
}
