// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public final class ArmPolicy{
    public static double armPower;
    public static double velocity;
    public static double low;
    public static double medium;
    public static double high;
    public static void presets(){
        low = 20;
        medium = 40;
        high = 60;
    }
}
