// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants.OperatorConstants
import frc.robot.commands.DevourCoralCommand
import frc.robot.commands.LaunchCoralCommand
import frc.robot.commands.gotoPoseCommand
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ElevatorSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.SwerveSubsystem
import swervelib.SwerveInputStream
import java.io.File
import java.util.*
import java.util.stream.Stream
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the [Robot] periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    val driverXbox: CommandXboxController = CommandXboxController(0)
    val operatorXbox: CommandXboxController = CommandXboxController(1)

     fun getScaleFactor(): Double {
        val angle = atan2(driverXbox.leftY, driverXbox.leftX) % (Math.PI/2);
        return sin((angle - Math.PI/4.0).absoluteValue + Math.PI/4.0)
    }

    // The robot's subsystems and commands are defined here...
    private val drivebase = SwerveSubsystem(
        File(
            Filesystem.getDeployDirectory(),
            "swerve/neo"
        )
    )

    private val armSubsystem = ArmSubsystem(
        if (RobotBase.isReal()) {
            ArmSubsystem.ArmNeoIO(Constants.Arm.motorId)
        } else {
            ArmSubsystem.ArmSimIO()
        })

    private val elevatorSubsystem = ElevatorSubsystem(
        if (RobotBase.isReal()) {
            ElevatorSubsystem.ElevatorNeoIO(Constants.Elevator.motorId)
        } else {
            ElevatorSubsystem.ElevatorSimIO()
        })

    private val intakeSubsystem = IntakeSubsystem(
        if (RobotBase.isReal()) {
            IntakeSubsystem.IntakeNeoIO(Constants.Intake.MOTOR)
        } else {
            IntakeSubsystem.IntakeSym()
        })

    private val isCompetition = true;

    private val autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        { stream: Stream<PathPlannerAuto> ->
            if (isCompetition) {
                stream.filter({auto:PathPlannerAuto -> auto.getName().startsWith("comp")})
            } else {
                stream
            }
        });


    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    var driveAngularVelocity: SwerveInputStream = SwerveInputStream.of(
        drivebase.swerveDrive,
        { driverXbox.leftY * -1 * getScaleFactor()},
        { driverXbox.leftX * -1 * getScaleFactor()})
        .withControllerRotationAxis { driverXbox.rightX }
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true)

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    var driveDirectAngle: SwerveInputStream = driveAngularVelocity.copy().withControllerHeadingAxis(
        { driverXbox.rightX * getScaleFactor()},
        { driverXbox.rightY * getScaleFactor()})
        .headingWhile(true)


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    var driveFieldOrientedDirectAngle: Command = drivebase.driveFieldOriented(driveDirectAngle)

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    var driveFieldOrientedAnglularVelocity: Command = drivebase.driveFieldOriented(driveAngularVelocity)

    var driveSetpointGen: Command = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle)



    var driveAngularVelocitySim: SwerveInputStream = SwerveInputStream.of(
        drivebase.swerveDrive,
        { -driverXbox.leftY * getScaleFactor()},
        { -driverXbox.leftX * getScaleFactor()})
        .withControllerRotationAxis { driverXbox.getRawAxis(2) }
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true)

    // Derive the heading axis with math!
    var driveDirectAngleSim: SwerveInputStream = driveAngularVelocitySim.copy()
        .withControllerHeadingAxis(
            {
                sin(
                    driverXbox.getRawAxis(
                        2
                    ) * Math.PI
                ) * (Math.PI * 2)
            },
            {
                cos(
                    driverXbox.getRawAxis(
                        2
                    ) * Math.PI
                ) *
                        (Math.PI * 2)
            })
        .headingWhile(true)

    var driveFieldOrientedDirectAngleSim: Command = drivebase.driveFieldOriented(driveDirectAngleSim)

    var driveSetpointGenSim: Command = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim)


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the trigger bindings
        configureBindings()
        DriverStation.silenceJoystickConnectionWarning(true)
        NamedCommands.registerCommand("launch_coral", LaunchCoralCommand(intakeSubsystem))
        NamedCommands.registerCommand("devour_coral", DevourCoralCommand(intakeSubsystem))
        NamedCommands.registerCommand("l1", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L1))
        NamedCommands.registerCommand("l2", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L2))
        NamedCommands.registerCommand("l3", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L3))
        NamedCommands.registerCommand("l4", gotoPoseCommand(armSubsystem,elevatorSubsystem,Constants.Poses.L4))

        SmartDashboard.putData("Auto Chooser", autoChooser)

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
        drivebase.defaultCommand =
            if (!RobotBase.isSimulation()) driveFieldOrientedDirectAngle else driveFieldOrientedDirectAngleSim

        if (RobotBase.isSimulation()) {
            driverXbox.start().onTrue(Commands.runOnce({
                drivebase.resetOdometry(
                    Pose2d(
                        3.0,
                        3.0,
                        Rotation2d()
                    )
                )
            }))
        }
        if (DriverStation.isTest()) {
            drivebase.defaultCommand = driveFieldOrientedAnglularVelocity // Overrides drive command above!

            driverXbox.b().whileTrue(drivebase.sysIdAngleMotorCommand())
            driverXbox.x().whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2))
            driverXbox.start().onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            driverXbox.back().whileTrue(drivebase.centerModulesCommand())
            driverXbox.leftBumper().onTrue(Commands.none())
            driverXbox.rightBumper().onTrue(Commands.none())
        } else {
            driverXbox.a().onTrue((Commands.runOnce({ drivebase.zeroGyro() })))
            driverXbox.x().onTrue(Commands.runOnce({ drivebase.addFakeVisionReading() }))
            driverXbox.b().whileTrue(
                drivebase.driveToPose(
                    Pose2d(Translation2d(4.0, 4.0), Rotation2d.fromDegrees(0.0))
                )
            )
            //driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2.0))
            driverXbox.start().whileTrue(Commands.none())
            driverXbox.back().whileTrue(Commands.none())
            driverXbox.leftBumper().whileTrue(Commands.runOnce({ drivebase.lock() }, drivebase).repeatedly())
            driverXbox.rightBumper().onTrue(Commands.none())

            operatorXbox.y().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L4))
            operatorXbox.b().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L3))
            operatorXbox.x().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L2))
            operatorXbox.a().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.L1))
            operatorXbox.povUp().onTrue(gotoPoseCommand(armSubsystem, elevatorSubsystem, Constants.Poses.Pickup))

            operatorXbox.rightBumper().whileTrue(LaunchCoralCommand(intakeSubsystem))
        }

        fun applyTeam(speed: Double):Double {
            if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Blue)) {
                return -speed;
            } else {
                return speed;
            }
        }

//        fun upMax(speed: Double):Double {
//            val sign = speed.sign;
//            return sign * Math.min(Math.abs(speed) - 0.1,1.0);
//        }

        val leftY =
                {
                    MathUtil.applyDeadband(
                        applyTeam(driverXbox.leftY),
                        Constants.OperatorConstants.LEFT_Y_DEADBAND
                    )
                }


        val leftX: () -> Double =
                {
                    MathUtil.applyDeadband(
                        applyTeam(driverXbox.leftX),
                        Constants.OperatorConstants.LEFT_X_DEADBAND
                    )
                };

        val omega = {
            MathUtil.applyDeadband(
                driverXbox.rightX,
                Constants.OperatorConstants.LEFT_X_DEADBAND
            )
        }

        val driveMode = { true }

        val simClosedFieldRel =
            TeleopDrive(
                drivebase,
                leftY,
                leftX,
                omega,
                driveMode,
            )

        drivebase.defaultCommand = simClosedFieldRel
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
