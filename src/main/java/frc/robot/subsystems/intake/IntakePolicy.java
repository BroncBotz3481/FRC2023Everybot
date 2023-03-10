// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public final class IntakePolicy {
    public static double intakePower;
    public static final double noGameElementSpeed = 1000;
    public static double encoderTicks1;
    public static double encoderTicks2;

    public boolean isConeHere() {
        if (encoderTicks1 < noGameElementSpeed)
            return true;
        return false;      
    }
    public boolean isCubeHere() {
        if (encoderTicks2 < noGameElementSpeed)
            return true;
        return false;
    }
}

