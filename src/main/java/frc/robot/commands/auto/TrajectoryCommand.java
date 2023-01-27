/****************************** Header ******************************\
 Class Name: ExampleCommand extends CommandBase
 File Name: ExampleCommand.java
 Summary: An example command to use for learning and testing.
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Dylan Watson
 Email: dylantrwatson@gmail.com
 \********************************************************************/


package frc.robot.commands.autoCommands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * An example command that uses an example subsystem.
 */
public class TrajectoryCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final Timer timer;
    private final RamseteController m_ramseteController;
    private Trajectory autoTrajectory;
    private Trajectory.State currentGoal;
    private ChassisSpeeds adjustedSpeeds;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private Pose2d start;
    private List<Translation2d> interiorWaypoints;
    private TrajectoryConfig config;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TrajectoryCommand(DrivetrainSubsystem subsystem) {

        timer = new Timer();
        m_ramseteController = new RamseteController();
        m_drivetrainSubsystem = subsystem;
        addRequirements(subsystem);
    }

    public void generateTrajectory(){
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics
        (Units.inchesToMeters(23.625));

        wheelSpeeds = new DifferentialDriveWheelSpeeds
        (DrivetrainPolicy.leftEncoderVelocity, DrivetrainPolicy.rightEncoderVelocity);

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

        // Linear velocity
        double linearVelocity = chassisSpeeds.vxMetersPerSecond;

        // Angular velocity
        double angularVelocity = chassisSpeeds.omegaRadiansPerSecond;

        // Step 1:
        config = new TrajectoryConfig(5, 5);
        config.setReversed(true);
        config.setEndVelocity(0);
        config.setStartVelocity(0);
        config.setKinematics(kinematics);

        // Step 2: Define starting point
        start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        // Step 3: Define points
        interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0, 0.5)); // forward .5m
//        interiorWaypoints.add(new Translation2d(-0.5, 0)); // left .5m
//        interiorWaypoints.add(new Translation2d(0,-0.5)); // backwards .5m
//        interiorWaypoints.add(new Translation2d(0.5,0)); // right .5m

        //forward, left, backward, right (1)

//        var Start = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0),
//        Rotation2d.fromDegrees(0));
//
//        var Point1 = new Pose2d(Units.feetToMeters(0.5), Units.feetToMeters(0),
//        Rotation2d.fromDegrees(30));
//


//
//
//        var trajectory1 = TrajectoryGenerator.generateTrajectory(Start,
//        interiorWaypoints, Point1, config);
//
//        var Point2 = new Pose2d(Units.feetToMeters(0.5+0.5*Math.sqrt(3)), Units.feetToMeters(0.25),
//        Rotation2d.fromDegrees(-30));
//
//        interiorWaypoints.add(new Translation2d(0.5,0));
//
//        var trajectory2 = TrajectoryGenerator.generateTrajectory(Point1,
//        interiorWaypoints, Point2, config);
//
//        var concatTrajectory = trajectory1.concatenate(trajectory2);
//
//        var End = new Pose2d(Units.feetToMeters(0.5+2*0.5*Math.sqrt(3)), Units.feetToMeters(0),
//        Rotation2d.fromDegrees(-30));
//
//        interiorWaypoints.add(new Translation2d(0.5,0));
//
//        var trajectory3 = TrajectoryGenerator.generateTrajectory(Point2,
//        interiorWaypoints, End, config);
//
//        autoTrajectory = concatTrajectory.concatenate(trajectory3);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        generateTrajectory();
        autoTrajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, new Pose2d(new Translation2d(0, 0.5), Rotation2d.fromDegrees(0)), config);
        timer.start();
        timer.reset();
        // Step 4: generate trajectory
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentGoal = autoTrajectory.sample(timer.get());
        adjustedSpeeds = m_ramseteController.calculate(DrivetrainPolicy.driveOdometry.getPoseMeters(), currentGoal);
        wheelSpeeds = DrivetrainPolicy.driveKinematics.toWheelSpeeds(adjustedSpeeds);
        //new DifferentialDriveWheelSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        m_drivetrainSubsystem.set(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.run(0, 0);
        timer.stop();
        timer.reset();
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= 4;
    }
}
