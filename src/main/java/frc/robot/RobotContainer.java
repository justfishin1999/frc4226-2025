// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.algaeRemover;
import frc.robot.commands.RunAlgaeIn;
import frc.robot.commands.RunAlgaeOut;
import frc.robot.commands.RunIntakeIn;
import frc.robot.commands.RunIntakeOut;



import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.wrist;


import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  private elevator m_elevator = new elevator();
  private wrist m_wrist = new wrist();
  private Intake m_intake = new Intake();
  private algaeRemover m_algaeRemover = new algaeRemover();

  private SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController secondaryXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,driverXbox::getRightY).headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //NamedCommands.registerCommand("elevatorLow", new InstantCommand(() -> m_elevator.low()));
    NamedCommands.registerCommand("output", new RunIntakeOut(m_intake));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings()
  {
    secondaryXbox.leftBumper().onTrue(new InstantCommand(() -> m_wrist.actuateWrist(0)));
    secondaryXbox.rightBumper().onTrue(new InstantCommand(() -> m_wrist.actuateWrist(0)));
    secondaryXbox.back().onTrue(new InstantCommand(() -> m_wrist.actuateWrist(0)));

    secondaryXbox.povDown().whileTrue(new RunIntakeIn(m_intake));
    secondaryXbox.povUp().whileTrue(new RunIntakeOut(m_intake));

    secondaryXbox.a().onTrue(new InstantCommand(() -> m_elevator.runElevator(30)));
    secondaryXbox.b().onTrue(new InstantCommand(() -> m_elevator.runElevator(6)));

    secondaryXbox.x().whileTrue(new RunAlgaeOut(m_algaeRemover));
    secondaryXbox.y().whileTrue(new RunAlgaeIn(m_algaeRemover));

    //new JoystickButton(secondaryXbox, Button.kRightBumper.value).whileTrue(new output(m_head));
    //new JoystickButton(secondaryXbox, Button.kLeftBumper.value).whileTrue(new intake(m_head));

    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("New Auto");
    //return new PathPlannerAuto("auto");
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
