/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.RobotContainerConstants.gamepadPort;
import static frc.robot.Constants.RobotContainerConstants.leftStickPort;
import static frc.robot.Constants.RobotContainerConstants.rightStickPort;
import static frc.robot.Constants.RouteFinderConstants.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.RouteFinderConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.RouteFinderConstants.kTrackwidthMeters;
import static frc.robot.Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.RouteFinderConstants.ksVolts;
import static frc.robot.Constants.RouteFinderConstants.kvVoltSecondsPerMeter;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import frc.robot.utils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.BallCameraConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Climber;
// import frc.robot.Constants.FeederConstants;
// import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavxGyro;
// import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Turret; 
// import frc.robot.subsystems.NavxGyro;
import edu.wpi.first.wpilibj.SPI;

public class RobotContainer {
  // Subsystems should be private here and have to be passed to commands because it is better coding practice.

  // Joysticks
  public static final Joystick leftStick = new Joystick(leftStickPort);
  public static final Joystick rightStick = new Joystick(rightStickPort);
  public static final Joystick gamepad = new Joystick(gamepadPort);

  // Compressor
  // public static final Compressor compressor = new Compressor(Constants.compressorModelPort, PneumaticsModuleType.REVPH);

  // Drivetrain
  public static final Drivetrain drivetrain = new Drivetrain();

  // Shooter
  public static final Shooter shooter = new Shooter();

  // Intake
  public static final Intake intake = new Intake();
  
  // Network Table
  public static NetworkTable py_vision_network_table;

  // //Climber
  public static final Climber climber = new Climber();

  //Gyro
  public static final NavxGyro navx = new NavxGyro(SPI.Port.kMXP);
  
  //Color Wheel
  // public static final ColorWheel colorwheel = new ColorWheel();
  // public static final ColorWheel colorwheel = null;

  //Trajectory
  public static Trajectory trajectory;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     /**
     * COMMAND INFO From the official wpilib docs: "While users are able to create
     * commands by explicitly writing command classes (either by subclassing
     * CommandBase or implementing Command), for many commands (such as those that
     * simply call a single subsystem method) this involves a lot of wasteful
     * boilerplate code. To help alleviate this, many of the prewritten commands
     * included in the command-based library may be inlined - that is, the command
     * body can be defined in a single line of code at command construction."
     * 
     * TLDR: You shouldn't create a whole new file for a command that only calls one
     * method.  This should only be lamda's for buttons. For subsystem conntrols look 
     * at drive tele for an example on how to do commands.
     */
  }

  public static TrajectoryConfig getConfig() {
    var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics, 10);

    // Create config for trajectory
    return new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        // Doesn't reverse the trajectory
        .setReversed(false);
  }

  /**
   * 
   * @return the command to run in autonomous
   */
  public static Ball getClosestBall() {
    py_vision_network_table = NetworkTableInstance.getDefault().getTable("pyVision");
    String jsonString = py_vision_network_table.getEntry("jsonData").getString("");
    GsonBuilder gsonBuilder = new GsonBuilder();
    Gson gson = gsonBuilder.create();
    Ball[] ball_array = gson.fromJson(jsonString, Ball[].class);
    // No JSON / no ball in sight
    if (ball_array == null || ball_array.length == 0) {
      return null;
    }
    // At least one ball - figure out closest ball
    // TODO: Get this value from start of game
    String teamColor = "red";
    Ball closestBall = null;
    for (Ball ball: ball_array) {
      // System.out.println(ball);
      if (closestBall == null) closestBall = ball;
      // Check if next ball is closer than current ball
      if (ball.getRadius() > closestBall.getRadius()) closestBall = ball;
    }
    return closestBall;
  }

  private double safeAutoTurnSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)
  private double safeAutoForwardSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)

  /*
  public Command ball_auto_sniffer() {
    // [PR] Description of this mutilated code - plz unmutilate it
    // Robot will start turning, looking around for ball of particular color within a certain radius range
    // It's probably between 1 and 6 feet but idk
    // Once it sees the closest ball it will stop turning and drive forward toward the ball
    // 
    Ball closestBall = getClosestBall();
    if (closestBall == null) {
      // TODO: Turn robot in a circle
      System.out.println("(ball_auto_sniffer) Turn robot in a circle");
      drivetrain.drive(-safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor(), safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor()); // Robot is rotating right
      return null;
    }
    // Check if need to turn - not needed if ball is almost in the center of the frame (2px away)
    if (Math.abs(closestBall.getCenterX()-BallCameraConstants.FRAME_WIDTH)/2 <= 2) {
      // If ball is close enough, stop moving closer - 450 is largest radius until we stop
      if (closestBall.getRadius() >= 450) {
        System.out.println("(ball_auto_sniffer) Ball is close enough, stop moving");
        drivetrain.stop();
      }
      // Move robot closer to robot
      else {
        System.out.println("(ball_auto_sniffer) Move robot forward in straight line to ball");
        drivetrain.drive(safeAutoForwardSpeed*drivetrain.getVoltageRegScaleFactor(), safeAutoForwardSpeed*drivetrain.getVoltageRegScaleFactor());
      }
    }
    // Need to turn
    else {
      // Figure out which direction to turn (left or right)
      if (closestBall.getCenterX() > BallCameraConstants.FRAME_WIDTH/2) {
        // Turn left
        System.out.println("(ball_auto_sniffer) Turn robot left to center ball");
        drivetrain.drive(safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor(), -safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor());
      } else {
        // Turn right
        System.out.println("(ball_auto_sniffer) Turn robot right to center ball");
        drivetrain.drive(-safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor(), safeAutoTurnSpeed*drivetrain.getVoltageRegScaleFactor());
      }
    }
    return null;
  }
  */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
    public Command getAutonomousCommand() {
      var kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

      String trajectoryJSON = "paths/tenFt_backforth_test.json";
      try {
        
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

        
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }

    // Create a voltage constraint to ensure we don't accelerate too fast
      DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.RouteFinderConstants.ksVolts,
                                      Constants.RouteFinderConstants.kvVoltSecondsPerMeter,
                                       Constants.RouteFinderConstants.kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);
    
     // Reset odometry, then run path following command, then stop at the end.
     drivetrain.resetOdometry(trajectory.getInitialPose());
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, // We input our desired trajectory here
        drivetrain::getPose, new RamseteController(Constants.RouteFinderConstants.kRamseteB, Constants.RouteFinderConstants.kRamseteZeta),
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), kDriveKinematics,
        drivetrain::getWheelSpeeds, new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0), new PIDController(Constants.RouteFinderConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts, drivetrain);

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
  */

}
