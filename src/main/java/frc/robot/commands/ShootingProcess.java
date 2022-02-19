// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootingProcess extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Joystick gamepad;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ShootingProcess(Shooter shooter, Joystick gamepad) {
    this.shooter = shooter;
    this.gamepad = gamepad;
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

    // // Gamepad left bumper button -> run shooter
    // if (gamepad.getRawButton(Constants.Left_Bumper_Button)) {
    //   // WORKING
    //   shooter.setRunning(true);
    //   shooter.getToSpeed();
    //   if (shooter.atSpeed()) {
    //     System.out.println("Ready to feed -> shoot");
    //   }
    // } else {
    //   shooter.stopShooter();
    //   shooter.setRunning(false);
    // }
    // // Gamepad left trigger -> run tower in
    // if (gamepad.getRawButton(Constants.Left_Trigger_Button)) {
    //   shooter.feed();
    // } else {
    //   shooter.stopFeeding();
    // }


    // // Shooter controls
    if (gamepad.getRawButton(Constants.Right_Trigger_Button)) {
      System.out.println("Right trigger pressed");
      shooter.setRunning(true);
      shooter.getToSpeed();
      if (shooter.atSpeed()) {
        System.out.println("Should be shooting now!");
        shooter.feed();
      } 
    } else {
      shooter.stop();
      shooter.setRunning(false);
    }

    if (!shooter.isRunning()) {
      if (gamepad.getRawButton(Constants.Left_Bumper_Button) || gamepad.getRawButton(Constants.Left_Trigger_Button)) {
        shooter.reverseFeed();
      } 
    }
    System.out.println("shooting process execute() code is running...");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
