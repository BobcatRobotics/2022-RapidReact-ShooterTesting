// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class shootBalls extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shootBalls(Shooter s, Intake i) {
    shooter = s;
    intake = i;
    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    shooter.stopFeeding();
    // shooter.getToSpeed();
    intake.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.getToSpeed();

    if (shooter.atSpeed()) {
      shooter.feed();
      shooter.setShooterSolenoidExtended(true);
      intake.runIntakeBarIn(true);
      intake.runIntakeWheelsIn(true);
    } else {
      shooter.stopFeeding();
      // shooter.getToSpeed();
      intake.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
    return false;
  }
}
