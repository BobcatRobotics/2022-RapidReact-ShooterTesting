package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;

public class Auto5BallTrajectories {

    public static TrajectoryConfig getConfig(boolean shouldReverseTrajectory) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            10);

        // Create config for trajectory
        return new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(AutoConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                // Should reverse the trajectory?
                .setReversed(shouldReverseTrajectory);
    }

    public static Trajectory moveToBall1() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.feetToMeters(1.5), 0)),
            new Pose2d(Units.feetToMeters(3), 0, new Rotation2d(0)),
            getConfig(false));
    }
    
    public static Trajectory moveForwardFromBall1() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(3), 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.feetToMeters(2), 0)),
            new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(0)),
            getConfig(true));
    }

    public static Trajectory moveToBall2() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.feetToMeters(2), Units.feetToMeters(-1))),
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            getConfig(false));
    }

    public static Trajectory moveToBall3() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            List.of(new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-1))),
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(260), new Rotation2d(80)),
            getConfig(false));
    }
        
    public static Trajectory moveToShootBall3() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(260), new Rotation2d(80)),
            List.of(new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-1))),
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            getConfig(false));
    }

}
