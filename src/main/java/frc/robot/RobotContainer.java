// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.arm.CompressArmCommand;
import frc.robot.commands.arm.ExtendArmHighCommand;
import frc.robot.commands.arm.ExtendArmLowCommand;
import frc.robot.commands.arm.ExtendArmMediumCommand;
import frc.robot.commands.arm.StopArmCommand;
import frc.robot.commands.auto.MultiPieceChargeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.OverdriveCommand;
import frc.robot.commands.intake.SpinCommand;
import frc.robot.commands.intake.SpitCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_driveTrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  private final XboxController m_operatorController =
      new XboxController(OperatorConstants.kDriverOperatorPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveTrainSubsystem.setDefaultCommand(new DriveCommand(m_driveTrainSubsystem,m_driverController::getLeftY, m_driverController::getRightY));
    m_armSubsystem.setDefaultCommand(new StopArmCommand(m_armSubsystem));
    m_intakeSubsystem.setDefaultCommand(new StopIntakeCommand(m_intakeSubsystem));

    new Trigger(m_operatorController::getAButton).whileTrue(new SpinCommand(m_intakeSubsystem));
    new Trigger(m_operatorController::getBButton).whileTrue(new SpitCommand(m_intakeSubsystem));
    new Trigger(m_operatorController::getXButton).whileTrue(new CompressArmCommand(m_armSubsystem));
    new Trigger(m_operatorController::getYButton).whileTrue(new ExtendArmHighCommand(m_armSubsystem));
    new Trigger(m_operatorController::getLeftBumper).whileTrue(new ExtendArmMediumCommand(m_armSubsystem));
    new Trigger(m_operatorController::getRightBumper).whileTrue(new ExtendArmLowCommand(m_armSubsystem));
    
    new Trigger(m_driverController::getRightStickButton).or(new Trigger(m_driverController::getLeftStickButton).whileTrue(new OverdriveCommand(m_driveTrainSubsystem, m_driverController::getLeftY, m_driverController::getRightY)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new MultiPieceChargeCommand();
  }
}
