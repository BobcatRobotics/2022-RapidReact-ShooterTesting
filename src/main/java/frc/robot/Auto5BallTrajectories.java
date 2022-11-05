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

    // Create a voltage constraint to ensure we don't accelerate too fast
    public static DifferentialDriveVoltageConstraint getAutoVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            10);
    }

    public static TrajectoryConfig getConfig(boolean shouldReverseTrajectory) {

        // Create config for trajectory
        return new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(AutoConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(getAutoVoltageConstraint())
                // Should reverse the trajectory?
                .setReversed(shouldReverseTrajectory);
    }

    public static Trajectory moveToBall1() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(-Units.feetToMeters(3), 0)),
            new Pose2d(-Units.feetToMeters(6), 0, new Rotation2d(0)),
            new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(AutoConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            AutoConstants.ksVolts,
                            AutoConstants.kvVoltSecondsPerMeter,
                            AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                    10))
                // Should reverse the trajectory?
                .setReversed(true));
    }
    
    public static Trajectory moveForwardFromBall1() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(3), 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.feetToMeters(2), 0)),
            new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(0)),
            getConfig(false));
    }

    public static Trajectory moveToBall2() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(1), 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.feetToMeters(2), Units.feetToMeters(-1))),
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            getConfig(true));
    }

    public static Trajectory moveToBall3() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            List.of(new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-1))),
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(260), new Rotation2d(80)),
            getConfig(true));
    }
        
    public static Trajectory moveToShootBall3() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(260), new Rotation2d(80)),
            List.of(new Translation2d(Units.feetToMeters(1), Units.feetToMeters(-1))),
            new Pose2d(Units.feetToMeters(0), Units.inchesToMeters(100), new Rotation2d(135)),
            getConfig(true));
    }

    public static Trajectory getCurvyTrajectory() {
        return TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          // new Pose2d(0, 0, new Rotation2d(-Math.PI/2)),
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through line
          List.of(
            new Translation2d(Units.feetToMeters(-1.5*3), 0),
            new Translation2d(Units.feetToMeters(-3*3), Units.feetToMeters(-2*1.25)),
            new Translation2d(Units.feetToMeters(-4.5*3), Units.feetToMeters(2*1.25)),
            new Translation2d(Units.feetToMeters(-6*3), Units.feetToMeters(0)),
            new Translation2d(Units.feetToMeters(-6.5*3), Units.feetToMeters(0))
          ),
          new Pose2d(Units.feetToMeters(-7.5*3), 0, new Rotation2d(0)),
          // Pass config
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                10))
            .setReversed(true));
          // use negative waypoints at end cuz coordinates relative 
      }
// X and Y are swapped in code vs reality and +Y is to the side of the 177 sign
    public static Trajectory moveForward() {
        return TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through line
          List.of(new Translation2d(Units.feetToMeters(7*1.35), 0),
                    new Translation2d(Units.feetToMeters(9*1.35), Units.feetToMeters(2*1.35))),
          // End 1 meters straight ahead of where we started, facing forward
          new Pose2d(Units.feetToMeters(14*1.35), 0, new Rotation2d(0)),
          // Pass config
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                10))
            .setReversed(false));
    }
// ON ENABLE, reset the gyro
// 1.35 distance multiplier and -10-15 degree error 11/5/2022
    public static Trajectory moveToBall() {
        return TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(Units.feetToMeters(14*1.35)+0, 0, new Rotation2d(0)),
          // Pass through line
          List.of(),
          // End 1 meters straight ahead of where we started, facing forward
          new Pose2d(Units.feetToMeters(14*1.35)-Units.feetToMeters(6), -Units.feetToMeters(6), new Rotation2d(Units.degreesToRadians(80))),
          // Pass config
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                10))
            .setReversed(true));
    }

    public static Trajectory moveFromBall() {
        return TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          // Pass through line
          new Pose2d(Units.feetToMeters(14*1.35)-Units.feetToMeters(6), -Units.feetToMeters(6), new Rotation2d(Units.degreesToRadians(80))),
          List.of(),
          new Pose2d(Units.feetToMeters(14*1.35)+0, 0, new Rotation2d(0)),
          // End 1 meters straight ahead of where we started, facing forward
          // Pass config
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                10))
            .setReversed(false));
    }

    public static Trajectory moveBackToStart() {
        return TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          // Pass through line
          new Pose2d(Units.feetToMeters(14*1.35)+0, 0, new Rotation2d(0)),
          List.of(new Translation2d(Units.feetToMeters(12*1.35), Units.feetToMeters(2*1.35)),
          new Translation2d(Units.feetToMeters(10*1.35), -Units.feetToMeters(2*1.35)),
          new Translation2d(Units.feetToMeters(8*1.35), Units.feetToMeters(2*1.35)),
          new Translation2d(Units.feetToMeters(6*1.35), -Units.feetToMeters(2*1.35)),
          new Translation2d(Units.feetToMeters(4*1.35), 0)),
          new Pose2d(0, 0, new Rotation2d(0)),
          // End 1 meters straight ahead of where we started, facing forward
          // Pass config
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                  AutoConstants.ksVolts,
                    AutoConstants.kvVoltSecondsPerMeter,
                    AutoConstants.kaVoltSecondsSquaredPerMeter),
                    AutoConstants.kDriveKinematics,
                10))
            .setReversed(false));
    }


}
