// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DriveTele;
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
  private ShootingProcess shootingProcess = new ShootingProcess(RobotContainer.shooter, RobotContainer.gamepad, RobotContainer.climber);
  private ClimberCommand climberCommand = new ClimberCommand(RobotContainer.climber, RobotContainer.rightStick, RobotContainer.gamepad);



  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private Joystick gamepad;
  private Intake intake;
  private Shooter shooter;
  private Climber climber;
  private Drivetrain drivetrain;
  private Compressor compressor;
  private boolean shoot = false;
  // private CommandBase desiredAutoCommand;
  private ShuffleboardTab tab = Shuffleboard.getTab("Things Tab");
  private double waitTime = 0;

  private boolean use_RS_Shift_Switch = true;

  // set to true to use Parallel Command Groups
  private boolean commandGroupTest = true;
  private ParallelCommandGroup commandGroup = new ParallelCommandGroup();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  // private Joystick rightStick;
  // private Joystick leftStick;



  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    gamepad = m_robotContainer.gamepad;
    intake = m_robotContainer.intake;
    shooter = m_robotContainer.shooter;
    climber = m_robotContainer.climber;
    // this.rightStick = RobotContainer.rightStick;
    // this.leftStick = RobotContainer.leftStick;

    drivetrain = m_robotContainer.drivetrain;
    // compressor = m_robotContainer.compressor;
    SmartDashboard.putNumber("Set Speed", 4400);
    tab.add("Shooter current RPM",0);
    // Add Shuffleboard toggle for switching between RS shift switch and A button
    SmartDashboard.putBoolean("Use RS shift switch?", true);
    SmartDashboard.putNumber("Left shooter current", shooter.getLeftCurrent());
    SmartDashboard.putNumber("Right shooter current", shooter.getRightCurrent());
    SmartDashboard.putNumber("Left shooter voltage", shooter.getLeftVoltage());
    SmartDashboard.putNumber("Right shooter voltage", shooter.getRightVoltage());
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());
    // commandGroup.addCommands(intakeControls,drivetele,shootingProcess);
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    intake.stopIntake();
    drivetrain.coast();
    // intake.deploy(false);
    shooter.stopShooter();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.deadAutoOne();

    // schedule the autonomous command (example)
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
    //   commandGroup.schedule();
    // } else {
      // drive controller
      drivetele.schedule();
      // shooter controller
      shootingProcess.schedule();
      // intake controller
      intakeControls.schedule();
      // climber controller
      climberCommand.schedule();

    // }

  }

  // double right = 0, left = 0;
  // double driveThreshold = 0.07;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // if (gamepad.getRawButton(Constants.Left_Trigger_Button)) {
    //   shooter.setRunning(true);
    //   shooter.getToSpeed();
    // } else {
    //   shooter.stopShooter();
    //   shooter.setRunning(false);
    // }
    // if (gamepad.getRawButton(Constants.Right_Bumper_Button)) {
    //   shooter.feed(true);
    // } else if (gamepad.getRawButton(Constants.Right_Trigger_Button)){
    //   shooter.reverseFeed();
    // } else {
    //   shooter.stop();
    // }
    
    // Turn on climber mode
    // if (use_RS_Shift_Switch ? gamepad.getRawButton(Constants.RS_Shift_Switch) : gamepad.getRawButtonReleased(Constants.A_Button))
    if (gamepad.getRawButtonReleased(Constants.A_Button)) {
      // Intake up
      intake.deploy(false);
      // Switch to climber mode
      climber.toggleSwitchToClimberMode();   
      // Turn off shooter if on climber mode
      if (climber.isClimberMode()) {
        shooter.setRunning(false);
        shooter.stop();
      }
    } else {
      // Reset shooter to lower hub shooting speed otherwise
      shooter.setRunning(false);
      shooter.getToSpeed();
    }

    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    double speed = SmartDashboard.getNumber("Set Speed", 4800);
    // System.out.println("SET SPEED IS " + speed);
    shooter.setSpeed(speed);
    SmartDashboard.putNumber("Current RPM", shooter.getLeftRPM());

    // Update button used to toggle climber mode based on Shuffleboard input
    use_RS_Shift_Switch = SmartDashboard.getBoolean("Use RS shift switch?", true);
  
    SmartDashboard.putNumber("Left shooter current", shooter.getLeftCurrent());
    SmartDashboard.putNumber("Right shooter current", shooter.getRightCurrent());
    SmartDashboard.putNumber("Left shooter voltage", shooter.getLeftVoltage());
    SmartDashboard.putNumber("Right shooter voltage", shooter.getRightVoltage());
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());

    // SmartDashboard.putNumber("Current RPM", shooter.getRightRPM());
    
    // SmartDashboard.putNumber("NavX heading cos", RobotContainer.navx.getRotation2d().getCos());
    // SmartDashboard.putNumber("NavX X Disp", RobotContainer.navx.getDisplacementX());
    // SmartDashboard.putNumber("NavX Y Disp", RobotContainer.navx.getDisplacementY());
    // SmartDashboard.putNumber("NavX Z Disp", RobotContainer.navx.getDisplacementZ());
    // SmartDashboard.putNumber("NavX angle", RobotContainer.navx.getAngle());
    SmartDashboard.putString("DriveTrain get pose", drivetrain.getPose().toString());
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
