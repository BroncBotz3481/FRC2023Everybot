// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public final class ArmPolicy{
    public static double armPower;
    public static double targetSpeed;
    public static double ticks;
    public static final double low = 50;
    public static final double medium = 100;
    public static final double high = 150;
    public static double clicksPerRev;

    public static void presets(){
    }
}
