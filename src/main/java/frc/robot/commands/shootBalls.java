// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class shootBalls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Intake intake;
  Timer t = new Timer();
  double time_alloted;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shootBalls(Shooter s, Intake i, double time_allotted) {
    shooter = s;
    intake = i;
    time_alloted = time_allotted;
    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    shooter.stopFeeding();
    shooter.setShootingModeKey(ShooterConstants.UPPER_HUB_KEY);
    // shooter.getToSpeed();
    intake.stopIntake();
    t.reset();
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!shooter.isShooterSolenoidExtended()){
    //   shooter.setShooterSolenoidExtended(true);
    // }
    if (t.hasElapsed(time_alloted)) {
      shooter.stop();
      intake.stopIntake();
    } else {
      shooter.getToSpeed();
  
      if (shooter.atSpeed()) {
        shooter.feed();
        intake.runIntakeBarIn(true);
        intake.runIntakeWheelsIn(true);
      } else {
        shooter.stopFeeding();
        // shooter.getToSpeed();
        intake.stopIntake();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    if(intake.isDeployed()){
      intake.deploy(false);
    }

    if (shooter.atSpeed()) {
      shooter.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (shooter.atSpeed() && intake.bar) check intake speeds and all that
    return t.hasElapsed(time_alloted);
  }
}
