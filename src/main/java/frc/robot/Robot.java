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
import frc.robot.Constants.ShooterConstants;
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

  private int selected_dead_auto_ID = 0;
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

    drivetrain = m_robotContainer.drivetrain;
    SmartDashboard.putNumber("Set High Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    SmartDashboard.putNumber("Set Low Speed", ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED);
    tab.add("Shooter current RPM",0);
    SmartDashboard.putBoolean("Use RS shift switch?", true);
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());
    SmartDashboard.putNumber("Compressor pressure", intake.pneumaticHub().getPressure(0));
    
    SmartDashboard.putNumber("Selected Dead Auto #", selected_dead_auto_ID);
    SmartDashboard.putString("Selected Dead Auto ID", m_robotContainer.deadAutoIDs[selected_dead_auto_ID]);
    SmartDashboard.putNumber("Delay time: Dead auto 2-ball", 0);
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
    // intake.deploy(true);
    shooter.stopShooter();
    updateShuffleBoard();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    updateShuffleBoard();
    
    
    // NEED TO TEST -----
    if (selected_dead_auto_ID == 1) {
      m_autonomousCommand = m_robotContainer.deadAuto_threeBall_right();
    } else {
      m_autonomousCommand = m_robotContainer.deadAuto_twoBall(Math.max(0.0, Math.round(SmartDashboard.getEntry("Delay time: Dead auto 2-ball").getDouble(0.0)*2)/2.0));
    }
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
    //   commandGroup.schedules();
    // } else {
      // drive controller
      drivetele.schedule();
      // shooter controller
      shootingProcess.schedule();
      // intake controller
      intakeControls.schedule();
      // climber controller
      climberCommand.schedule();
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
      shooter.setShooterSolenoidExtended(false);
      // Turn off shooter if on climber mode
      if (climber.isClimberMode()) {
        shooter.setRunning(false);
        shooter.stop();
      }
    } else {
      // Reset shooter to lower hub shooting speed otherwise
      shooter.setRunning(false);
      shooter.stop();
      // shooter.getToSpeed();
    }

    updateShuffleBoard();
  }

  public void updateShuffleBoard() {
    double highSpeed = SmartDashboard.getNumber("Set High Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    double hoodHigh = SmartDashboard.getNumber("Set High Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    double highSpeed = SmartDashboard.getNumber("Set High Speed", ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED);
    double lowSpeed = SmartDashboard.getNumber("Set Low Speed", ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED);
    // // System.out.println("SET SPEED IS " + speed);
    shooter.setHighSpeed(highSpeed);
    shooter.setLowSpeed(lowSpeed);
    SmartDashboard.putNumber("Current RPM", shooter.getLeftRPM());

    // Update button used to toggle climber mode based on Shuffleboard input
    use_RS_Shift_Switch = SmartDashboard.getBoolean("Use RS shift switch?", true);
    SmartDashboard.putBoolean("Is climber mode on?", climber.isClimberMode());
    SmartDashboard.putString("DriveTrain get pose", drivetrain.getPose().toString());
// NEED TO TEST -----
    selected_dead_auto_ID = (int)SmartDashboard.getNumber("Selected Dead Auto #", 0);
    if (selected_dead_auto_ID != 0 && selected_dead_auto_ID != 1) selected_dead_auto_ID = 0;
    SmartDashboard.putString("Selected Dead Auto ID", m_robotContainer.deadAutoIDs[selected_dead_auto_ID]);
    // NEED TO TEST -----
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
