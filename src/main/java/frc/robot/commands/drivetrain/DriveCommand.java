// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final DoubleSupplier m_leftpower, m_rightpower;
  /**
   * @param subsystem
   *  Creates a new DriveCommand. */
  public DriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier leftPower, DoubleSupplier rightPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = subsystem;
    m_leftpower = leftPower;
    m_rightpower = rightPower;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivetrainSubsystem.run( m_leftpower.getAsDouble(), m_rightpower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.run(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
