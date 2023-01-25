// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeMotor1;
  private final CANSparkMax intakeMotor2;
  private final RelativeEncoder motor1Encoder;
  private final RelativeEncoder motor2Encoder;

  public IntakeSubsystem() {
    intakeMotor1 = new CANSparkMax(4, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(5, MotorType.kBrushless);
    intakeMotor1.setInverted(false);
    intakeMotor2.setInverted(true);
    motor1Encoder = intakeMotor1.getEncoder();
    motor2Encoder = intakeMotor2.getEncoder();
  }

  public void runIntake(double power){
    IntakePolicy.intakePower = power;
    intakeMotor1.set(IntakePolicy.intakePower);
    intakeMotor2.set(IntakePolicy.intakePower);
  }

  public void stopIntake(){
    runIntake(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
