// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/** Add your docs here. */
package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import frc.robot.Constants;

public final class DrivetrainPolicy {
    public static double powerLeft; // power for left motors
    public static double powerRight; // power for right motors
    public static double leftSpeed;
    public static double rightSpeed;
    public static DifferentialDriveOdometry driveOdometry;
    public static DifferentialDriveKinematics driveKinematics;

    public static double rightEncoderPosition = 0, rightEncoderVelocity = 0;
    public static double leftEncoderPosition = 0, leftEncoderVelocity = 0;

//For Orry
    
    //RPM
    public static double getRightVelocity() {
        return ((DrivetrainPolicy.rightSpeed * 60) / (Math.PI * Constants.DriveTrainConstants.wheelDiameter));
    }
    //RPM
    public static double getLeftVelocity() {
        return ((DrivetrainPolicy.leftSpeed * 60) / (Math.PI * Constants.DriveTrainConstants.wheelDiameter));

    }


}
