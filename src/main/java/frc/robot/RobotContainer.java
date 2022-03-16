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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BallCameraConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.driveCommand;
import frc.robot.commands.dropAndSuck;
import frc.robot.commands.shootBalls;
import frc.robot.commands.turnDegreeCommand;
import frc.robot.commands.waitCommand;
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
  // public static Ball getClosestBall() {
  //   py_vision_network_table = NetworkTableInstance.getDefault().getTable("pyVision");
  //   String jsonString = py_vision_network_table.getEntry("jsonData").getString("");
  //   GsonBuilder gsonBuilder = new GsonBuilder();
  //   Gson gson = gsonBuilder.create();
  //   Ball[] ball_array = gson.fromJson(jsonString, Ball[].class);
  //   // No JSON / no ball in sight
  //   if (ball_array == null || ball_array.length == 0) {
  //     return null;
  //   }
  //   // At least one ball - figure out closest ball
  //   // TODO: Get this value from start of game
  //   String teamColor = "red";
  //   Ball closestBall = null;
  //   for (Ball ball: ball_array) {
  //     // // System.out.println(ball);
  //     if (closestBall == null) closestBall = ball;
  //     // Check if next ball is closer than current ball
  //     if (ball.getRadius() > closestBall.getRadius()) closestBall = ball;
  //   }
  //   return closestBall;
  // }

  // private double safeAutoTurnSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)
  // private double safeAutoForwardSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)

  //One of the dead autos(if our auto is bricked :( ))

  public String[] deadAutoIDs = {"dA_2B", "dA_3B_right"};

  public SequentialCommandGroup deadAuto_twoBall(double startingWaitTime) {
    //Drive forward setCommandVelocity = 1 meter/s
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    Command wait = new waitCommand(startingWaitTime);
    Command drive = new driveCommand(drivetrain, 4, 4, 0.9);
    Command dns = new dropAndSuck(intake);
    Command shoot = new shootBalls(shooter, intake, 5);
    Command dummy = new driveCommand(drivetrain, 4, 4, 1);
    commandGroup.addCommands(wait,dns,drive,shoot,dummy);
    return commandGroup;
  }

  public SequentialCommandGroup deadAuto_threeBall_right() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Drive back, intake & shoot
    Command dns1 = new dropAndSuck(intake);
    Command dns2 = new dropAndSuck(intake);
    Command driveBackwards1 = new driveCommand(drivetrain, 4, 4, 0.9);
    // Command driveBackwards2 = new driveCommand(drivetrain, 4, 4, 0.9);
    Command shoot1 = new shootBalls(shooter, intake, 3);
    Command shoot2 = new shootBalls(shooter, intake, 3);
    
    // Drive forward a little bit to avoid hitting the wall when turning
    Command driveAdjust = new driveCommand(drivetrain, -2, -2, 0.3);

    // Turn deg
    // brute force turn
    // Command turn = new driveCommand(drivetrain, -4, 4, 0.62);
    Command turn = new turnDegreeCommand(drivetrain, 101.5);

    // drive forward & intake
    Command driveBackwards2 =  new driveCommand(drivetrain, 4, 4, 2);
    // add dns in commandGroup
    // turn back
    // brute force turn
    Command turnBack = new turnDegreeCommand(drivetrain, -52);
    Command driveForward2 =  new driveCommand(drivetrain, -3, -3, 0.35);
    // shoot
    // add shoot in command group
    commandGroup.addCommands(dns1, driveBackwards1, shoot1, driveAdjust, turn, dns2, driveBackwards2, turnBack, driveForward2, shoot2);
    return commandGroup;
  }

  // public SequentialCommandGroup deadAutoTwo() {
  //   SequentialCommandGroup commandGroup = new SequentialCommandGroup();

  //   // Drive back, intake & shoot
  //   Command dns1 = new dropAndSuck(intake);
  //   Command dns2 = new dropAndSuck(intake);
  //   Command driveBackwards1 = new driveCommand(drivetrain, 4, 4, 0.9);
  //   Command driveBackwards2 = new driveCommand(drivetrain, 4, 4, 0.9);
  //   Command shoot1 = new shootBalls(shooter, intake, 3);
  //   Command shoot2 = new shootBalls(shooter, intake, 3);
    
  //   // Drive forward a little bit to avoid hitting the wall when turning
  //   Command driveAdjust = new driveCommand(drivetrain, -2, -2, 0.3);

  //   // Turn deg
  //   // brute force turn
  //   Command turn = new driveCommand(drivetrain, -4, 4, 0.62);

  //   // drive forward & intake
  //   Command driveForward =  new driveCommand(drivetrain, 4, 4, 2);
  //   // add dns in commandGroup

  //   // turn back
  //   // brute force turn
  //   Command turnBack = new driveCommand(drivetrain, 4, -4, 0.28);

  //   // shoot
  //   // add shoot in command group

  //   commandGroup.addCommands(dns1, driveBackwards1, shoot1, driveAdjust, turn, dns2, driveForward, turnBack, shoot2, driveBackwards2);
  //   return commandGroup;
  // }

  // public SequentialCommandGroup deadAutoThree() {
  //   SequentialCommandGroup commandGroup = new SequentialCommandGroup();
  //   Command turn90Command = new turnDegreeCommand(drivetrain, 90);
  //   // 93 deg, -75 deg or signs flip
  //   commandGroup.addCommands(turn90Command);
  //   return commandGroup;
  // }
}
