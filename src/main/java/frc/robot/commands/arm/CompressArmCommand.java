// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.ArmPolicy;
import frc.robot.subsystems.arm.ArmSubsystem;

public class CompressArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;
  private final DoubleSupplier m_power;
  /** Creates a new CompressArmCommand. 
   * @param subsystem
  */

  public CompressArmCommand(ArmSubsystem subsystem, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmSubsystem = subsystem;
    m_power = power;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmPolicy.armPower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double finalPos = ArmPolicy.ticks*2;
    if(ArmPolicy.ticks != finalPos)
      m_ArmSubsystem.pidMove(-12000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
