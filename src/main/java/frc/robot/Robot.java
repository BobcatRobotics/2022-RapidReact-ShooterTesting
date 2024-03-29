// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.CenterRobotOnHub;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveTele;
import frc.robot.commands.LEDControl;
import frc.robot.commands.ShootingProcess;
import frc.robot.commands.intakeControls;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // commands and crap
  private intakeControls intakeControls = new intakeControls(RobotContainer.intake,  RobotContainer.gamepad, RobotContainer.shooter, RobotContainer.climber);
  private DriveTele drivetele = new DriveTele(RobotContainer.drivetrain, RobotContainer.rightStick, RobotContainer.leftStick);
  private ArcadeDrive arcadedrivetele = new ArcadeDrive(RobotContainer.drivetrain, RobotContainer.rightStick, RobotContainer.leftStick);
  private ShootingProcess shootingProcess = new ShootingProcess(RobotContainer.shooter, RobotContainer.intake, RobotContainer.gamepad, RobotContainer.climber, RobotContainer.limelight);
  private ClimberCommand climberCommand = new ClimberCommand(RobotContainer.climber, RobotContainer.rightStick, RobotContainer.gamepad);
  private CenterRobotOnHub centerRobotOnHubCommand = new CenterRobotOnHub(RobotContainer.drivetrain, RobotContainer.gamepad, RobotContainer.limelight);
  // private LEDControl ledControl = new LEDControl(RobotContainer.ledLights);

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private Joystick gamepad;
  private Intake intake;
  private Shooter shooter;
  private Climber climber;
  private Drivetrain drivetrain;
  private Limelight limelight;
  private Compressor compressor;
  private boolean shoot = false;
  // private CommandBase desiredAutoCommand;
  private ShuffleboardTab tab = Shuffleboard.getTab("Things Tab");
  private double waitTime = 0;
  private boolean autoMode = false;

  private boolean use_RS_Shift_Switch = true;

  // set to true to use Parallel Command Groups
  private boolean commandGroupTest = true;
  private ParallelCommandGroup commandGroup = new ParallelCommandGroup();

  private int selected_dead_auto_ID = 0;

  private final SendableChooser<Command> driveChooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    CameraServer.startAutomaticCapture();

    //  lower fps
    m_robotContainer = new RobotContainer();
    gamepad = m_robotContainer.gamepad;
    intake = m_robotContainer.intake;
    shooter = m_robotContainer.shooter;
    climber = m_robotContainer.climber;
    limelight = m_robotContainer.limelight;
    // climber.withdraw();
    // this.rightStick = RobotContainer.rightStick;
    // this.leftStick = RobotContainer.leftStick;

    drivetrain = m_robotContainer.drivetrain;
    // limelight.turnOffLED();
    // limelight.turnOnLED();

    // Port forwarding
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
    PortForwarder.add(8888, "wpilibpi.local", 80);

    // compressor = m_robotContainer.compressor;
    SmartDashboard.putBoolean("did deploy", true);
    SmartDashboard.putNumber("Set High Speed", ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(0.0)[0]);
    SmartDashboard.putNumber("Set Low Speed", ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(0.5)[0]);
    SmartDashboard.putNumber("Set High Hood Speed", ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(0.0)[1]);
    SmartDashboard.putNumber("Set Low Hood Speed", ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(0.5)[1]);
    tab.add("Shooter current RPM",0);
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());
    SmartDashboard.putNumber("Compressor pressure", intake.pneumaticHub().getPressure(0));
    SmartDashboard.putNumber("Main shooter RPM Threshold", shooter.getMainRPMThreshold());
    SmartDashboard.putNumber("Hood shooter RPM Threshold", shooter.getHoodRPMThreshold());
    SmartDashboard.putString("Team Color", "red");
    
    
    SmartDashboard.putNumber("Selected Dead Auto #", selected_dead_auto_ID);
    SmartDashboard.putString("Selected Dead Auto ID", m_robotContainer.deadAutoIDs[selected_dead_auto_ID]);
    SmartDashboard.putNumber("Delay time: Dead auto 2-ball", 0);
    SmartDashboard.putNumber("Limelight dist (m)", 0);

    driveChooser.setDefaultOption("arcade", arcadedrivetele);
    driveChooser.addOption("tank", drivetele);
    // SmartDashboard.putNumber("Gyro heading", 0);
    // SmartDashboard.putString("JsonString", "STARTING");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(driveChooser);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    limelight.turnOnLED();
    updateShuffleBoard();
  }

  @Override
  public void disabledPeriodic() {
    intake.stopIntake();
    drivetrain.coast();
    // intake.deploy(true);
    shooter.stopShooter();
    updateShuffleBoard();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoMode = true;
    limelight.turnOnLED();
    updateShuffleBoard();
    

    if (selected_dead_auto_ID == 1) {
      m_autonomousCommand = m_robotContainer.deadAuto_threeBall_right();
    } else if (selected_dead_auto_ID ==0 ) {
      m_autonomousCommand = m_robotContainer.deadAuto_twoBall(Math.max(0.0, Math.round(SmartDashboard.getEntry("Delay time: Dead auto 2-ball").getDouble(0.0)*2)/2.0));
    } else if (selected_dead_auto_ID == 3) {
      m_autonomousCommand = m_robotContainer.deadAuto_fiveBall();
    } else if (selected_dead_auto_ID == 4) {
      m_autonomousCommand = m_robotContainer.deadAuto_FourBall();
    } else if (selected_dead_auto_ID == 5) {
      m_autonomousCommand = m_robotContainer.deadAuto_fiveBall_2();
    } else {
      m_autonomousCommand = m_robotContainer.centerBallOnTargetAuto();
    }
    

    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    updateShuffleBoard();

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    autoMode = false;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().cancelAll();
    
    // We will most likely want to use command group test as this will allow
    // our threads to run together and not step on each others toes.
    // Think of a command group as a threadpool and a command as its own thread
    // sequential group will run commands one after another, this would operate like a set of threads with semaphores (might look chooppy)
    // parrallel group will run commands in parrallel duhhh
    // parallel race will run all commands at the same time but kill all of them when one finishes (race conditions)
    // parallel deadline will run all commands in parallel but will end them when a specific command gets executed
    // if(commandGroupTest) {
    //   commandGroup.schedules();
    // } else {
      // drive controller
      // drivetele.schedule();
      driveChooser.getSelected().schedule();
      // shooter controller
      shootingProcess.schedule();
      // intake controller
      intakeControls.schedule();
      // climber controller
      climberCommand.schedule();
      // centerRobotOnHub command
      centerRobotOnHubCommand.schedule(false);
      // leds
      // ledControl.schedule();

      climber.turnOffClimberMode();
    // }

  }

  // double right = 0, left = 0;
  // double driveThreshold = 0.07;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (!drivetrain.isBrake()) {
      drivetrain.brake();
    }
    CommandScheduler.getInstance().run();
  
    // Turn on climber mode
    // if (use_RS_Shift_Switch ? gamepad.getRawButton(Constants.RS_Shift_Switch) : gamepad.getRawButtonReleased(Constants.A_Button))
    if (gamepad.getRawButtonReleased(Constants.A_Button)) {
      // Intake up
      intake.deploy(false);
      // Switch to climber mode
      
      climber.toggleSwitchToClimberMode();
      // Lift hood up
      // Turn off shooter if on climber mode
      if (climber.isClimberMode()) {
        // shooter.setShooterSolenoidExtended(false);
        shooter.setRunning(false);
        shooter.stop();
      } else {
        // shooter.setShooterSolenoidExtended(true);
        
      }
    }

    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    double highSpeed = SmartDashboard.getNumber("Set High Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    double lowSpeed = SmartDashboard.getNumber("Set Low Speed", ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED);
    double hoodhigh = SmartDashboard.getNumber("Set High Hood Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    double hoodlow = SmartDashboard.getNumber("Set Low Hood Speed", ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED);
    // // System.out.println("SET SPEED IS " + speed);
    shooter.setHighSpeed(highSpeed);
    shooter.setLowSpeed(lowSpeed);
    shooter.setHighHoodSpeed(hoodhigh);
    shooter.setLowHoodSpeed(hoodlow);
    SmartDashboard.putNumber("Current Left RPM", shooter.getLeftRPM());
    SmartDashboard.putBoolean("AutoMode", autoMode);
    

    NetworkTable py_vision_network_table = NetworkTableInstance.getDefault().getTable("pyVision");
    String jsonString = py_vision_network_table.getEntry("jsonData").getString("NO_DATA");
    // SmartDashboard.putString("JsonString", jsonString);
    // SmartDashboard.putNumber("Current Right RPM", shooter.getRightRPM());
    SmartDashboard.putNumber("Hood RPM", shooter.getHoodRPM());
    SmartDashboard.putNumber("Limelight dist (m)", (LimelightConstants.kLimelightHeight / Math.tan(limelight.y()*Math.PI/180 + LimelightConstants.kLimelightMountAngle)));
    // SmartDashboard.putNumber("LIMELIGHT WORK PLZ (y)", limelight.y());

    if (autoMode) {
      shooter.setMainRPMThreshold(SmartDashboard.getNumber("Main shooter RPM Threshold", 50+100));
      shooter.setHoodRPMThreshold(SmartDashboard.getNumber("Hood shooter RPM Threshold", 50+100));
    } else {
      shooter.setMainRPMThreshold(SmartDashboard.getNumber("Main shooter RPM Threshold", 50));
      shooter.setHoodRPMThreshold(SmartDashboard.getNumber("Hood shooter RPM Threshold", 50));
    }
    // Update button used to toggle climber mode based on Shuffleboard input
    // use_RS_Shift_Switch = SmartDashboard.getBoolean("Use RS shift switch?", true);
    String colour = SmartDashboard.getString("Team Color","red");
    m_robotContainer.setTeamColor(colour);
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());
    // SmartDashboard.putNumber("Gyro heading", Rotation2d.fromDegrees(drivetrain.getHeading()).getDegrees());
    // SmartDashboard.putString("DriveTrain get pose", drivetrain.getPose().toString());
    // SmartDashboard.putNumber("Gyro heading", drivetrain.getHeading());
    
    // NetworkTable py_vision_network_table = NetworkTableInstance.getDefault().getTable("pyVision");
    // String jsonString = py_vision_network_table.getEntry("jsonData").getString("");
    // SmartDashboard.putString("PyvisString", jsonString);

    // NEED TO TEST -----
    selected_dead_auto_ID = (int)SmartDashboard.getNumber("Selected Dead Auto #", 0);
    if (!(0 <= selected_dead_auto_ID && selected_dead_auto_ID <= 1)) selected_dead_auto_ID = 0;
    SmartDashboard.putString("Selected Dead Auto ID", m_robotContainer.deadAutoIDs[selected_dead_auto_ID]);
  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}