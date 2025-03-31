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
import org.photonvision.PhotonCamera
import CameraAlignInfo
import VisionSubsystem
import closestTag
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.RepeatCommand
import fieldLayout
import frc.robot.utils.PIDController
import getClosestPickupTag
import pathPlannerToReef
import kotlin.math.*
import kotlin.math.PI
import pathPlannerToPickup
import kotlin.jvm.optionals.getOrNull

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
        val pov = driverRightStick.getHID().getPOV()
        // SmartDashboard.putNumber("squared input magnitude", driverLeftStick.getX().pow(2.0) + driverLeftStick.getY().pow(2.0))
        // val angle = atan2(driveLeftStick.getY(), driverLeftStick.getX()) % (Math.PI/2.0);
        val factor = /*sin((angle - Math.PI/4.0).absoluteValue + Math.PI/4.0) * */
        if (pov == 0) {
            (1.0)//.pow((1/3))
        } else if (pov == 180) {
            (1.0/Constants.Drivebase.MAX_SPEED)
        } else {
            3.0/4.0
        } * if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Blue)) {
            -1.0
        } else {
            1.0
        };

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
            GrapplingSubsystem.GrapplingNeoIO(Constants.Grappling.motorId, Constants.Grappling.motor2Id)
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
                    drivebase,
                    PhotonCamera("ATFrontRight"), 
                    Transform3d(Inches.of(10.327), Inches.of(/*11.550*/-8.289), Inches.of(6.632), Rotation3d(0.0, 10.0 * (PI/180.0), 0.0)) ,
                    PhotonCamera("ATFrontLeft"),
                    Transform3d(Inches.of(10.327), Inches.of(8.289), Inches.of(6.632), Rotation3d(0.0, 10.0 * (PI/180.0), 0.0)) ,
//                     PhotonCamera("ATBack"),
//                     Transform3d(Inches.of(-5.793), Inches.of(0.223), Inches.of(39.098), Rotation3d(0.0, 30.0 * (PI/180.0), PI)),
                )
                    
            } else {
                VisionSubsystem.VisionSimIO()
            })

    private val isCompetition = true;

    private val autoChooser: SendableChooser<Command>;

    private var angle = 0.0;
    private var magnitude = 0.0;

    private var driveAngleSnap = false;

    private var ix = 0.0;
    private var iy = 0.0;

    var lastUpdatedInputs = -1;

    fun getIntputY(): Double {
        if (lastUpdatedInputs != frame) {
            iy = driverLeftStick.getY()
            ix = driverLeftStick.getX()
            val hypot = min(iy.pow(2.0) + ix.pow(2.0), 1.0)
            val angle = atan2(iy, ix)
            iy = abs(sin(angle) * hypot) * iy.sign

        }

        return iy
    }
    fun getIntputX(): Double {
        if (lastUpdatedInputs != frame) {
            iy = driverLeftStick.getY()
            ix = driverLeftStick.getX()
            val hypot = min((iy.pow(2.0) + ix.pow(2.0)), 1.0)
            val angle = atan2(iy, ix)
            ix = abs(cos(angle) * hypot) * ix.sign
        }


        return ix
    }


    private fun getMagnitude(): Double {
        val x = getIntputX();
        val y = getIntputY();
        return sqrt(x * x + y * y)
    }

    private fun getSnappedAngle(): Double {
        val x = getIntputX();
        val y = getIntputY();
        val rawAngle = atan2(y,x);
        return round(rawAngle/ Math.PI * 6.0) * Math.PI / 6.0
    }

    var faceAngle: Double? = null

    val rotationPid = PIDController(Constants.Drivebase.ROTATION_PID_TELEOP)

    // /**
    //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    //  */
    val driverXLimiter = SlewRateLimiter(30.0, -30.0, 0.0)
    val driverYLimiter = SlewRateLimiter(30.0, -30.0, 0.0)
    val driverRotationLimiter = SlewRateLimiter(32.0)
    var driveAngularVelocity: SwerveInputStream = SwerveInputStream.of(
        drivebase.swerveDrive,
        {
            val y = getIntputY()//driverLeftStick.getY()
            SmartDashboard.putNumber("joystick y", y)
            val yi = y * /*y.sign */getScaleFactor();
            driverYLimiter.calculate(abs(yi)) * yi.sign
//            yi * 0.5
        },
        {
            val x = getIntputX()//driverLeftStick.getX()
            SmartDashboard.putNumber("joystick x", x)
            val xi = x * /*x.sign * */getScaleFactor();
            driverXLimiter.calculate(abs(xi)) * xi.sign
//            xi * 0.5
        }
    )
        .withControllerRotationAxis {
            if (faceAngle != null) {
                SmartDashboard.putNumber("desired drivebase angle", faceAngle!! * 180.0/ Math.PI);
                SmartDashboard.putNumber("current drivebase angle", drivebase.heading.degrees)
                -rotationPid.calculate(drivebase.heading.degrees, faceAngle!! * 180.0/ Math.PI) / Constants.Drivebase.MAX_TURNING_SPEEDS
            } else {
                val i = driverRotationLimiter.calculate(driverRightStick.getX());
                i.pow(2) * i.sign * -1.2 * 0.95 * 0.75
            }
        }
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(1.0)
        .allianceRelativeControl(false)

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


        SmartDashboard.putNumber("LEFT_OFFSET_INCHES", Constants.Field.LEFT_OFFSET.`in`(Inches))
        SmartDashboard.putNumber("RIGHT_OFFSET_INCHES", Constants.Field.RIGHT_OFFSET.`in`(Inches))
        SmartDashboard.putData("Update field offset configuration", InstantCommand({
            Constants.Field.LEFT_OFFSET = Inches.of(SmartDashboard.getNumber("LEFT_OFFSET_INCHES", Constants.Field.LEFT_OFFSET.`in`(Inches)))
            Constants.Field.RIGHT_OFFSET = Inches.of(SmartDashboard.getNumber("RIGHT_OFFSET_INCHES", Constants.Field.RIGHT_OFFSET.`in`(Inches)))
            SmartDashboard.putNumber("LEFT_OFFSET_INCHES", Constants.Field.LEFT_OFFSET.`in`(Inches))
            SmartDashboard.putNumber("RIGHT_OFFSET_INCHES", Constants.Field.RIGHT_OFFSET.`in`(Inches))
        }))


        NamedCommands.registerCommand("launch_coral", launch_coral(intakeSubsystem,armSubsystem,elevatorSubsystem))
        NamedCommands.registerCommand("devour_coral", DevourCoralCommand(intakeSubsystem, true))
        NamedCommands.registerCommand("l1", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L1))
        NamedCommands.registerCommand("l2", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L2))
        NamedCommands.registerCommand("l3_algae", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L3Algae))
        NamedCommands.registerCommand("l2_algae", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L2Algae))

        NamedCommands.registerCommand("score_algae", ScoreAlgae(intakeSubsystem, armSubsystem, elevatorSubsystem))
        NamedCommands.registerCommand("flick_algae", FlickAlgae(intakeSubsystem, armSubsystem))


        NamedCommands.registerCommand("devour_algae", DevourAlgaeCommand(intakeSubsystem, true))
        NamedCommands.registerCommand("spin_up", InstantCommand({intakeSubsystem.setVoltageAlgae(Constants.Intake.INTAKE_ALGAE)}))


        NamedCommands.registerCommand("l3", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L3))
        NamedCommands.registerCommand("zero_pose", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.Zero))
        NamedCommands.registerCommand("barge_pose", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.Barge))


        NamedCommands.registerCommand("pickup_pose", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.Pickup))
        NamedCommands.registerCommand("l4", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L4))

        rotationPid.enableContinuousInput(-180.0, 180.0)

        // NamedCommands.registerCommand("align_first", alignReefSelect(drivebase, visionSubsystem, "align_first_left"));
        // NamedCommands.registerCommand("align_second", alignReefSelect(drivebase, visionSubsystem, "align_second_left"));
        // NamedCommands.registerCommand("align_third", alignReefSelect(drivebase, visionSubsystem, "align_third_left"));
        // NamedCommands.registerCommand("align_fourth", alignReefSelect(drivebase, visionSubsystem, "align_fourth_left"));

//         NamedCommands.registerCommand("align_pickup", ParallelCommandGroup(
//             gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Pickup),
//             pathPlannerToPickup(Inches.of(0.0), drivebase, true)
//         ))

        fun teamSelect(red: Int, blue: Int): Int {
            val alliance = DriverStation.getAlliance()

            val redteam = if (alliance.isPresent) {
                alliance.get() == Alliance.Red
            } else {
                true
            };

            return if (redteam) {
                red
            } else {
                blue
            }

        }

        for ((redtag, tag) in arrayOf(
            Pair(10, {teamSelect(10, 21)}),
            Pair(7, {teamSelect(7, 18)}),
            Pair(11, {teamSelect(11, 20)}),
            Pair(9, {teamSelect(9, 22)}),
            Pair(6, {teamSelect(6,19)}),
            Pair(8, {teamSelect(8, 17)}),
        )) {

             NamedCommands.registerCommand("align_right_${redtag}", pathPlannerToReef(Constants.Field.RIGHT_OFFSET, tag, drivebase, true))
             NamedCommands.registerCommand("align_left_${redtag}", pathPlannerToReef(Constants.Field.LEFT_OFFSET, tag, drivebase, true))
        }
        //NamedCommands.registerCommand("align_left_${10}", AlignReef(drivebase, visionSubsystem, Inches.of(6.5 - 0.5) , {10}, true))

        drivebase.setupPathPlanner()

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            { stream: Stream<PathPlannerAuto> ->
                if (isCompetition) {
                    stream.filter { auto: PathPlannerAuto ->
                        DriverStation.reportWarning("generating auto ${auto.getName()}", false);
                        auto.getName().startsWith("comp") }
                } else {
                    stream
                } 
            });

        SmartDashboard.putData("Auto Chooser", autoChooser)

    }

    private fun getSelectedTag(): Int {
        val alliance = DriverStation.getAlliance()

        val redteam = if (alliance.isPresent) {
            alliance.get() == Alliance.Red
        } else {
            true
        };

        fun teamSelect(red: Int, blue: Int): Int {
            return if (redteam) {
                red
            } else {
                blue
            }

        }

        if (driverLeftStick.getHID().getPOV() == 0) {
            return teamSelect(10, 21)
        }
        if (driverLeftStick.getHID().getPOV() == 180) {
            return teamSelect(7, 18)
        }
        if (driverLeftStick.getHID().getRawButton(5)) {
            return teamSelect(11, 20)
        }
        if (driverLeftStick.getHID().getRawButton(6)) {
            return teamSelect(9, 22)
        }
        if (driverLeftStick.getHID().getRawButton(3)) {
            return teamSelect(6,19)
        }
        if (driverLeftStick.getHID().getRawButton(4)) {
            return teamSelect(8, 17)
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
            // driverXbox.back().whileTrue(drivebase.cen`terModulesCommand())
            // driverXbox.leftBumper().onTrue(Commands.none())
            // driverXbox.rightBumper().onTrue(Commands.none())
        } else {
            driverLeftStick.button(7).onTrue(InstantCommand({armSubsystem.changeOffset(360.0/42.0/2.0)}))
            driverLeftStick.button(9).onTrue(InstantCommand({armSubsystem.changeOffset(-360.0/42.0/2.0)}))


            driverRightStick.button(11).onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            driverRightStick.button(9).whileTrue(MoveGrappleCommand(grappleSubsystem, -0.65))
            driverRightStick.button(10).whileTrue(MoveGrappleCommand(grappleSubsystem, -0.25))
            driverRightStick.button(8).whileTrue(MoveGrappleCommand(grappleSubsystem, 0.25))

            driverLeftStick.button(10).onTrue(InstantCommand({ grappleSubsystem.hold() }))

            driverRightStick.button(7).onTrue(RetractClimber(grappleSubsystem, 5.0))

            driverRightStick.button(12).onTrue(
                ParallelCommandGroup(
                    gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.CLIMB_POSE)
                        .andThen(GotoHeight(elevatorSubsystem, Meters.of(0.170))),
                    MoveOutClimberCommand(grappleSubsystem)
                ))

            driverLeftStick.button(12).onTrue(
                ParallelCommandGroup(
                    gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.CLIMB_POSE)
                    .andThen(GotoHeight(elevatorSubsystem, Meters.of(0.170))),
                    MoveOutClimberCommand(grappleSubsystem)
                    ))

            driverRightStick.button(3).whileTrue(
                RepeatCommand(InstantCommand({
                    val tag = closestTag(drivebase)
                    val location = fieldLayout.getTagPose(tag).getOrNull()
                    faceAngle = location?.rotation?.z
                })).finallyDo({ a ->
                    faceAngle = null;
                })
            )



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
             driverRightStick.button(1).whileTrue(pathPlannerToReef(Constants.Field.RIGHT_OFFSET/*, {getSelectedTag()}*/, drivebase, false))
             driverLeftStick.button(1).whileTrue(pathPlannerToReef(Constants.Field.LEFT_OFFSET/*, {getSelectedTag()}*/, drivebase, false))
             driverRightStick.button(2).whileTrue(pathPlannerToReef(Inches.of(0.0)/*, {getSelectedTag()}*/,drivebase, false))

            driverRightStick.button(6).whileTrue(RobotRelativeStrafeCommand(drivebase, -0.25))
            driverRightStick.button(5).whileTrue(RobotRelativeStrafeCommand(drivebase, 0.25))

//            driverLeftStick.button(2).onTrue(InstantCommand { })

//            driverLeftStick.button(2).whileTrue(
//
//            )
            driverRightStick.button(2).whileTrue(
                RepeatCommand(InstantCommand({
                    val tag = getClosestPickupTag(drivebase)
                    val location = fieldLayout.getTagPose(tag).getOrNull()
                    faceAngle = location?.rotation?.z
                })).finallyDo({ a ->
                    faceAngle = null;
                })
            )

//            driverLeftStick.button(7).onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.FullExtend))

            driverRightStick.button(3).onTrue(InstantCommand({
                setMotorBrake(true)
            }))
            driverRightStick.button(3).onFalse(InstantCommand({
                setMotorBrake(false)
            }))

            operatorXbox.rightStick().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L2NEW))

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

           operatorXbox.leftTrigger(0.1).onTrue(launch_coral(intakeSubsystem,armSubsystem, elevatorSubsystem).andThen(BackupCommand(drivebase)))
           operatorXbox.rightTrigger(0.1)./*onTrue*/whileTrue(DevourCoralCommand(intakeSubsystem,false))
           operatorXbox.rightBumper()./*onTrue*/whileTrue(DevourAlgaeCommand(intakeSubsystem, false))

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
