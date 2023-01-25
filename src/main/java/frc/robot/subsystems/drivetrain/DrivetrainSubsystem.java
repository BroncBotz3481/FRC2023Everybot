// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final CANSparkMax motorLeftFront;
  private final CANSparkMax motorRightFront;
  private final CANSparkMax motorLeftBack;
  private final CANSparkMax motorRightBack;
  private final SlewRateLimiter filter;

  private final DifferentialDrive driveTrain;

  private final SparkMaxPIDController leftPIDController;
  private final SparkMaxPIDController rightPIDController;

  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    motorLeftFront = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainLeftFrontCANID,MotorType.kBrushless);
    motorLeftBack = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainLeftBackCANID,MotorType.kBrushless);
    motorRightFront = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainRightFrontCANID,MotorType.kBrushless);
    motorRightBack = new CANSparkMax(Constants.DriveTrainConstants.kDrivetrainRightBackCANID,MotorType.kBrushless);
    filter = new SlewRateLimiter(.5);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);

    motorLeftFront.setInverted(false);
    motorRightBack.setInverted(true);

    driveTrain = new DifferentialDrive(motorLeftFront, motorRightFront);

    leftPIDController = motorLeftFront.getPIDController();
    rightPIDController = motorRightFront.getPIDController();

    frontLeftEncoder = motorLeftFront.getEncoder();
    frontRightEncoder = motorRightFront.getEncoder();



    this.setPIDF(0.01, 0, 0, 0, 200);
  }

  public void setPIDF(double P, double I, double D, double F, double integralZone){
    leftPIDController.setP(P);
    leftPIDController.setI(I);
    leftPIDController.setD(D);
    leftPIDController.setFF(F);
    leftPIDController.setIZone(integralZone);

    rightPIDController.setP(P);
    rightPIDController.setI(I);
    rightPIDController.setD(D);
    rightPIDController.setFF(F);
    rightPIDController.setIZone(integralZone);
}

  public void set(double leftSpeed, double rightSpeed) {
    DrivetrainPolicy.leftSpeed = leftSpeed;
    DrivetrainPolicy.rightSpeed = rightSpeed;

    leftPIDController.setReference(filter.calculate(DrivetrainPolicy.leftSpeed),ControlType.kVelocity);
    rightPIDController.setReference(filter.calculate(DrivetrainPolicy.rightSpeed), ControlType.kVelocity);
}

  public void run(double powerLeft, double powerRight){
    DrivetrainPolicy.powerLeft = powerLeft;
    DrivetrainPolicy.powerRight = powerRight;
    driveTrain.arcadeDrive(filter.calculate(DrivetrainPolicy.powerLeft), filter.calculate(DrivetrainPolicy.powerRight));
  }

  public void overDrive(double powerLeft, double powerRight){
    DrivetrainPolicy.powerLeft = powerLeft;
    DrivetrainPolicy.powerRight = powerRight;
    driveTrain.arcadeDrive(DrivetrainPolicy.powerLeft, DrivetrainPolicy.powerRight);
  }

  public void stopMotor(){
    run(0,0);
  }




  @Override
  public void periodic() {
    DrivetrainPolicy.leftEncoderPosition = frontLeftEncoder.getPosition();
    DrivetrainPolicy.rightEncoderPosition = frontRightEncoder.getPosition();
    DrivetrainPolicy.leftEncoderVelocity = frontLeftEncoder.getVelocity();
    DrivetrainPolicy.rightEncoderVelocity = frontRightEncoder.getVelocity();
  }
  
}
