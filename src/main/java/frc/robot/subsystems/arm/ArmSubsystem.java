// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax armMotor;
  private final SparkMaxPIDController PIDController;
  private final RelativeEncoder encoder;
  public ArmSubsystem() {
    armMotor = new CANSparkMax(ArmConstants.kArmMotorCANID, MotorType.kBrushless);
    PIDController = armMotor.getPIDController();
    encoder = armMotor.getEncoder();
    set(0,0,0,0,0);
  }
///
  public void set(double p, double i, double d, double f, double iz) {
    PIDController.setP(p);
    PIDController.setI(i);
    PIDController.setD(d);
    PIDController.setFF(f);
    PIDController.setIZone(iz);
  }

  public void moveArm(double power) {
    ArmPolicy.armPower = power;
    armMotor.set(ArmPolicy.armPower);
  }

  public void pidMove(double targetSpeed) {
    ArmPolicy.targetSpeed = targetSpeed;
    PIDController.setReference(ArmPolicy.targetSpeed, ControlType.kVelocity);
    //ArmPolicy.presets();
  }

  public void stopArm() {
    moveArm(0);
  }

  @Override
  public void periodic() {
    ArmPolicy.ticks = encoder.getVelocity()/encoder.getVelocityConversionFactor();
    ArmPolicy.clicksPerRev = encoder.getCountsPerRevolution();
  }
}
