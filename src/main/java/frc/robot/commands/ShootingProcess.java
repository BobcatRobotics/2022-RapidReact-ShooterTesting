// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootingProcess extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Joystick gamepad;
  private final Climber climber;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ShootingProcess(Shooter shooter, Joystick gamepad, Climber climber) {
    this.shooter = shooter;
    this.gamepad = gamepad;
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {                           // what the fuck is this sensor for
    // boolean[] tofArray = {shooter.getBallReadyToFeed(),shooter.getBallLeaving()};

    // Gamepad D-pad right -> shooter solenoid up
    if (gamepad.getPOV() == Constants.D_Pad_Right && !climber.isClimberMode()) {
      if (!shooter.isShooterSolenoidExtended()) {
        // System.out.println("D-pad right - shooter solenoid up");
        shooter.setShooterSolenoidExtended(true);
      }
    }
    // Gamepad D-pad left -> shooter solenoid down
    else if (gamepad.getPOV() == Constants.D_Pad_Left && !climber.isClimberMode()) {
      if (shooter.isShooterSolenoidExtended()) {
        // System.out.println("D-pad left - shooter solenoid down");
        shooter.setShooterSolenoidExtended(false);
      }
    }

    // Gamepad left bumper button -> run shooter high speed
    if (gamepad.getRawButton(Constants.Left_Bumper_Button)) {
      shooter.setHighMode(true);
      shooter.setRunning(true);
      shooter.getToSpeed();
    }
    // Gamepad D pad up -> run shooter low speed
    else if (gamepad.getPOV() == Constants.D_Pad_Up) {
      shooter.setHighMode(false);
      shooter.setRunning(true);
      shooter.getToSpeed();
    }
    // Stop shooter
    else {
      shooter.stopShooter();
      shooter.setRunning(false);
      // If climber mode, shooter should default to stop motor if button not pressed
      // if (climber.isClimberMode()) {
      //   shooter.stopShooter();
      // }
    }
    // Gamepad left trigger -> run tower in
    if (gamepad.getRawButton(Constants.Left_Trigger_Button)) {
      if (shooter.atSpeed()) {
        shooter.feed();
      }
    } else {
      shooter.stopFeeding();
    }

    if (gamepad.getRawButton(Constants.Right_Joystick_Pressed)) {
      shooter.reverseFeed();
    }

    // if (gamepad.getPOV() == Constants.D_Pad_Up) {
    //   shooter.setHighMode(true);
    // } else if (gamepad.getPOV() == Constants.D_Pad_Down) {
    //   shooter.setHighMode(false);
    // }

    //


    // // Shooter controls
    // if (gamepad.getRawButton(Constants.Right_Trigger_Button)) {
    //   // System.out.println("Right trigger pressed");
    //   shooter.setRunning(true);
    //   shooter.getToSpeed();
    //   if (shooter.atSpeed()) {
    //     // System.out.println("Should be shooting now!");
    //     shooter.feed();
    //   } 
    // } else {
    //   shooter.stop();
    //   shooter.setRunning(false);
    // }

    // if (!shooter.isRunning()) {
    //   if (gamepad.getRawButton(Constants.Left_Bumper_Button) || gamepad.getRawButton(Constants.Left_Trigger_Button)) {
    //     shooter.reverseFeed();
    //   } 
    // }
    // // System.out.println("shooting process execute() code is running...");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
