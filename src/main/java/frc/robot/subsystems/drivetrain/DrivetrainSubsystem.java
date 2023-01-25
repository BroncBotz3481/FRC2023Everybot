// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  private final CANSparkMax motorLeftFront;
  private final CANSparkMax motorRightFront;
  private final CANSparkMax motorLeftBack;
  private final CANSparkMax motorRightBack;
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    motorLeftFront = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainLeftFrontCANID,MotorType.kBrushless);
    motorLeftBack = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainLeftBackCANID,MotorType.kBrushless);
    motorRightFront = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainRightFrontCANID,MotorType.kBrushless);
    motorRightBack = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainRightBackCANID,MotorType.kBrushless);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);
  }

  public void runMotor(double powerLeft, double powerRight){
    DrivetrainPolicy.powerLeft = powerLeft;
    DrivetrainPolicy.powerRight = powerRight;
    motorLeftFront.set(DrivetrainPolicy.powerLeft);
    motorRightFront.set(DrivetrainPolicy.powerRight);
  }

  public void stopMotor(){
    runMotor(0,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
