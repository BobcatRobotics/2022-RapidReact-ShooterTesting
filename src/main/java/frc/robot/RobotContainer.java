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

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import frc.robot.utils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CenterRobotOnHub;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.alignToNearestBall;
import frc.robot.commands.driveCommand;
import frc.robot.commands.dropAndSuck;
import frc.robot.commands.shootBalls;
import frc.robot.commands.turnDegreeCommand;
import frc.robot.commands.waitCommand;
import frc.robot.commands.five_ball_auto_commands.FV_AlignToNearestBall;
import frc.robot.commands.five_ball_auto_commands.FV_DriveTime;
import frc.robot.commands.five_ball_auto_commands.FV_IntakeDownAndSuck;
import frc.robot.commands.five_ball_auto_commands.FV_LimelightHubAlign;
import frc.robot.commands.five_ball_auto_commands.FV_LimelightShoot;
import frc.robot.commands.five_ball_auto_commands.FV_PIDTurn;
import frc.robot.subsystems.Climber;
// import frc.robot.Constants.FeederConstants;
// import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
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

  
  public static Limelight limelight = new Limelight();
  // Shooter
  public static final Shooter shooter = new Shooter();

  // Intake
  public static final Intake intake = new Intake();
  
  // Network Table
  public static NetworkTable py_vision_network_table;

  // //Climber
  public static final Climber climber = new Climber();

  // Gyro
  public static final NavxGyro navx = new NavxGyro(SPI.Port.kMXP);

  // LEDs
  public static final LEDLights ledLights = new LEDLights(Constants.LED_port);
  
  //Color Wheel
  // public static final ColorWheel colorwheel = new ColorWheel();
  // public static final ColorWheel colorwheel = null;

  //Trajectory
  public static Trajectory trajectory;

  private static String teamColor = "red";
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

  public void setTeamColor(String color) {
    teamColor = color;
  }

  public String getTeamColor(){
    return teamColor;
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
    // No JSON / no ball in sights
    if (ball_array == null || ball_array.length == 0) {
      return null;
    }
    // At least one ball - figure out closest ball
    // TODO: Get this value from start of game
    Ball closestBall = null;
    for (Ball ball: ball_array) {
      // // System.out.println(ball);
      if (ball.getColor().equals(teamColor)) {
        if (closestBall == null ) 
          closestBall = ball;
        // Check if next ball is closer than current ball
        if (ball.getRadius() > closestBall.getRadius()) 
          closestBall = ball;
      }
    }
    return closestBall;
  }

  // private double safeAutoTurnSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)
  // private double safeAutoForwardSpeed = 0.1; // TODO: Tune this (I'm guessing it's between -1 and 1)

  //One of the dead autos(if our auto is bricked :( ))

  // public String[] deadAutoIDs = {"dA_2B", "dA_3B_right", "target", "dA_5B", "dA_4B", "dA_5B_2"};
  public String[] deadAutoIDs = {"dA_2B", "dA_3B_right"};
  
  public SequentialCommandGroup deadAuto_twoBall(double startingWaitTime) {
    //Drive forward setCommandVelocity = 1 meter/s
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    Command wait = new waitCommand(startingWaitTime);
    Command waitBeforeShooting = new waitCommand(1);
    Command drive = new driveCommand(drivetrain, 3.5, 3.5, 1.1);
    Command dns = new dropAndSuck(intake);
    // Command shoot = new shootBalls(shooter, intake, 5, false);
    
    Command shoot1 = new FV_LimelightShoot(limelight, shooter, intake, 5, false, 1);
    Command dummy = new driveCommand(drivetrain, 4, 4, 1);
    commandGroup.addCommands(wait,dns,drive,waitBeforeShooting,shoot1,dummy);
    return commandGroup;
  }

  public SequentialCommandGroup deadAuto_threeBall_right() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Drive back, intake & shoot
    Command dns1 = new dropAndSuck(intake);
    Command waitABit = new waitCommand(0.5);
    Command dns2 = new dropAndSuck(intake);
    Command driveBackwards1 = new driveCommand(drivetrain, 4, 4, 0.95);
    // Command driveBackwards2 = new driveCommand(drivetrain, 4, 4, 0.9);
    // Command shoot1 = new shootBalls(shooter, intake, 3,true);
    // Command shoot2 = new shootBalls(shooter, intake, 3,false);
    Command shoot1 = new FV_LimelightShoot(limelight, shooter, intake, 4, false, 1);
    Command shoot2 = new FV_LimelightShoot(limelight, shooter, intake, 4, false, 0.5);
    
    // Drive forward a little bit to avoid hitting the wall when turning
    Command driveAdjust = new driveCommand(drivetrain, -2, -2, 0.3);

    // Turn deg
    // brute force turn
    // Command turn = new driveCommand(drivetrain, -4, 4, 0.62);
    // Command turn = new turnDegreeCommand(drivetrain, 101.5);
    Command turn = new FV_PIDTurn(drivetrain, 111);
    // Command c = new alignToNearestBall(drivetrain);

    // drive forward & intake
    Command driveBackABitBeforeAligningToThirdBall = new driveCommand(drivetrain, 5, 5, 1.65);
    // Command driveBackwards2 =  new driveCommand(drivetrain, 6, 6, 1.15);
    // add dns in commandGroup
    // turn back
    // brute force turn
    // Command turnBack = new turnDegreeCommand(drivetrain, -52);
    Command turnBack = new FV_PIDTurn(drivetrain, -52);

    // Command alignToBall = new alignToNearestBall(drivetrain);
    // Command alignToBall2 = new alignToNearestBall(drivetrain);
    // Command driveForward2 =  new driveCommand(drivetrain, -3, -3, 0.35);
    
    Command waitABitMore = new waitCommand(0.25);
    Command centerBot = new CenterRobotOnHub(drivetrain, gamepad, limelight);
    // shoot
    // add shoot in command group
    
    commandGroup.addCommands(dns1, waitABit, driveBackwards1, shoot1, driveAdjust, turn, dns2, driveBackABitBeforeAligningToThirdBall, turnBack, waitABitMore, centerBot, shoot2);
    return commandGroup;
  }

  public SequentialCommandGroup centerBallOnTargetAuto () {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    Command c = new alignToNearestBall(drivetrain);
    Command d = new alignToNearestBall(drivetrain);

    commandGroup.addCommands(c,d);
    return commandGroup;

  }

  public SequentialCommandGroup deadAuto_fiveBall() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Ball 1-2
    Command intakeDownAndSuckFirstBall = new FV_IntakeDownAndSuck(intake);
    Command waitABit = new waitCommand(0.25);
    Command driveToFirstBall = new FV_DriveTime(drivetrain, 4, 4, 0.95);
    Command shootFirstBall = new FV_LimelightShoot(limelight, shooter, intake, 3, false, 0.5);
    Command driveForwardAfterShootingFirstBall = new FV_DriveTime(drivetrain, -2, -2, 0.3);

    // Ball 3
    Command turnToThirdBall = new FV_PIDTurn(drivetrain, 101.5);
    Command alignToThirdBall = new alignToNearestBall(drivetrain);
    Command intakeDownAndSuckThirdBall = new FV_IntakeDownAndSuck(intake);
    Command driveRestToThirdBall = new FV_DriveTime(drivetrain, 5, 5, 1.5);
    Command turnPartiallyToHubForThirdBall = new turnDegreeCommand(drivetrain, -52);
    Command alignToHubToShootThirdBall = new CenterRobotOnHub(drivetrain, gamepad, limelight);
    // Command limelightShootThirdBall = new FV_LimelightShoot(limelight, shooter, intake, 2, true);
    Command limelightShootThirdBall = new shootBalls(shooter, intake, 2, false);

    // Last balls
    Command turnToLastBalls = new FV_PIDTurn(drivetrain, 25);
    Command drivePartiallyToLastBalls = new FV_DriveTime(drivetrain, 5, 5.8, 2.5);
    Command alignToLastBalls = new alignToNearestBall(drivetrain);
    Command intakeDownAndSuckLastBalls = new FV_IntakeDownAndSuck(intake);
    Command finishDrivingToLastBalls = new FV_DriveTime(drivetrain, 2, 2, 2);
    Command driveToHubToShootLastBalls = new FV_DriveTime(drivetrain, -5, -5, 2);
    Command turnPartiallyToHubForLastBalls = new FV_PIDTurn(drivetrain, -25);
    Command alignToHubToShootLastBalls = new CenterRobotOnHub(drivetrain, gamepad, limelight);
    Command shootLastBalls = new FV_LimelightShoot(limelight, shooter, intake, 2, true, 0.5);

    // Add everything to the command group
    commandGroup.addCommands(intakeDownAndSuckFirstBall, waitABit, driveToFirstBall, shootFirstBall, driveForwardAfterShootingFirstBall, turnToThirdBall, alignToThirdBall, intakeDownAndSuckThirdBall, driveRestToThirdBall, turnPartiallyToHubForThirdBall, alignToHubToShootThirdBall, limelightShootThirdBall, turnToLastBalls, drivePartiallyToLastBalls, alignToLastBalls, intakeDownAndSuckLastBalls, finishDrivingToLastBalls, driveToHubToShootLastBalls, turnPartiallyToHubForLastBalls, alignToHubToShootLastBalls, shootLastBalls);

    return commandGroup;
  }

  public SequentialCommandGroup deadAuto_FourBall() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Ball 1-2
    Command intakeDownAndSuckFirstBall = new FV_IntakeDownAndSuck(intake);
    Command driveToFirstBall = new FV_DriveTime(drivetrain, 5, 5, 0.8);
    Command limelightAlignFirstBall = new FV_LimelightHubAlign(drivetrain, limelight);
    Command shootFirstBall = new FV_LimelightShoot(limelight, shooter, intake, 3, true, 0.5);
    
    // Ball 3-4
    Command driveToLastBalls = new FV_DriveTime(drivetrain, 4, 5, 0.8);
    Command alignToLastBalls = new FV_AlignToNearestBall(drivetrain);
    Command alignToLastBalls2 = new FV_AlignToNearestBall(drivetrain);
    Command intakeDownAndSuckLastBalls = new FV_IntakeDownAndSuck(intake);
    Command driveToPickUpLastBalls = new FV_DriveTime(drivetrain, 2, 2, 1);
    Command driveToHubToShootLastBalls = new FV_DriveTime(drivetrain, -4.5, -5, 0.8);
    Command alignToHubToShootLastBalls = new FV_LimelightHubAlign(drivetrain, limelight);
    Command shootLastBalls = new FV_LimelightShoot(limelight, shooter, intake, 3, true, 0.5);

    // Add all commands
    commandGroup.addCommands(intakeDownAndSuckFirstBall, driveToFirstBall, limelightAlignFirstBall, shootFirstBall, driveToLastBalls, alignToLastBalls, alignToLastBalls2, intakeDownAndSuckLastBalls, driveToPickUpLastBalls, driveToHubToShootLastBalls, alignToHubToShootLastBalls, shootLastBalls);
    return commandGroup;
  }

  public SequentialCommandGroup deadAuto_fiveBall_2() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Ball 1-2
    Command intakeDownAndSuckFirstBall = new FV_IntakeDownAndSuck(intake);
    Command driveToFirstBall = new FV_DriveTime(drivetrain, 5, 5, 0.8);
    Command limelightAlignFirstBall = new FV_LimelightHubAlign(drivetrain, limelight);
    Command shootFirstBall = new FV_LimelightShoot(limelight, shooter, intake, 3, false, 0.5);

    // Ball 3
    Command driveSteerForwardBeforeThirdBall = new FV_DriveTime(drivetrain, -3, -0.5, 0.5);
    Command alignToThirdBall = new FV_AlignToNearestBall(drivetrain);
    Command alignToThirdBall2 = new FV_AlignToNearestBall(drivetrain);
    Command intakeDownAndSuckThirdBall = new FV_IntakeDownAndSuck(intake);
    Command driveToThirdBall = new FV_DriveTime(drivetrain, 5, 5, 2);
    Command turnPartiallyToHubToShootThirdBall = new FV_PIDTurn(drivetrain, -50);
    Command alignToHubToShootThirdBall = new FV_LimelightHubAlign(drivetrain, limelight);
    Command shootThirdBall = new FV_LimelightShoot(limelight, shooter, intake, 2, true, 0.5);

    // Ball 4-5
    Command driveSteerToLastBalls = new FV_DriveTime(drivetrain, -4.5, -5, 2);
    Command alignToLastBalls = new FV_AlignToNearestBall(drivetrain);
    Command alignToLastBalls2 = new FV_AlignToNearestBall(drivetrain);
    Command intakeDownAndSuckLastBalls = new FV_IntakeDownAndSuck(intake);
    Command driveToLastBalls = new FV_DriveTime(drivetrain, 2.5, 2.5, 2);
    Command driveSteerToHubToShootLastBalls = new FV_DriveTime(drivetrain, 5, 5.6, 2);
    Command alignToHubToShootLastBalls = new FV_LimelightHubAlign(drivetrain, limelight);
    Command shootLastBalls = new FV_LimelightShoot(limelight, shooter, intake, 2, true, 0.5);

    // Add all commands
    commandGroup.addCommands(intakeDownAndSuckFirstBall, driveToFirstBall, limelightAlignFirstBall, shootFirstBall, driveSteerForwardBeforeThirdBall, alignToThirdBall, alignToThirdBall2, intakeDownAndSuckThirdBall, driveToThirdBall, turnPartiallyToHubToShootThirdBall, alignToHubToShootThirdBall, shootThirdBall, driveSteerToLastBalls, alignToLastBalls, alignToLastBalls2, intakeDownAndSuckLastBalls, driveToLastBalls, driveSteerToHubToShootLastBalls, alignToHubToShootLastBalls, shootLastBalls);
    return commandGroup;
  }
  public SequentialCommandGroup SeqParallelCommandGroupTest(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    Command DriveDistanceForward = new FV_DriveTime(drivetrain, 3, 3, 2);
    Command IntakeDown = new FV_IntakeDownAndSuck(intake);
    Command DriveDistanceBack = new FV_DriveTime(drivetrain, -3, -3, 1);
    Command IntakeDown2 = new FV_IntakeDownAndSuck(intake);
    Command turn10 = new FV_PIDTurn(drivetrain, 5);
    commandGroup.addCommands(Commands.parallel(DriveDistanceForward, IntakeDown), Commands.parallel(DriveDistanceBack, IntakeDown2));
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
