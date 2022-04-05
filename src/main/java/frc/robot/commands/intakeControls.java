// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class intakeControls extends CommandBase {

  private Intake intake;
  private Climber climber;
  private Joystick gp;

  public intakeControls(Intake in, Joystick gp, Shooter shoot, Climber climber) {
    this.intake = in;
    this.gp = gp;

    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gamepad X button -> intake down
    if (gp.getRawButton(Constants.X_Button) && !climber.isClimberMode()) {
      if (intake.isDeployed()) {
        intake.deploy(false);
      }
    }
    // Gamepad Y button -> intake up
    else if (gp.getRawButton(Constants.Y_Button) && !climber.isClimberMode()) {
      if (!intake.isDeployed()) {
        intake.deploy(true);
      }
    }

    // Gamepad B button -> intake + vertical wheels out
    if (gp.getRawButton(Constants.B_Button)) {
      intake.feedOut();
    } else {
      // Gamepad right bumper -> intake bar in, vertical wheels in
      if (gp.getRawButton(Constants.Right_Bumper_Button)) {
          intake.runIntakeBarIn(true);
          intake.runIntakeWheelsIn(true);
        // Also run in feed motor until TOF triggered
        // feed_in_until_TOF_triggered_or_while_shoot_button_pressed();
      } else {
        // Gamepad D-pad down -> intake out
        if (gp.getPOV() == Constants.D_Pad_Down) {
          intake.runIntakeBarOut(true);
        } else { // Stop intake bar
          intake.stopIntakeBar();
        }

        // Gamepad right trigger -> vertical wheels in
        if (gp.getRawButton(Constants.Right_Trigger_Button)) {
          intake.runIntakeWheelsIn(true);
          // Also run in feed motor until TOF triggered
          // feed_in_until_TOF_triggered_or_while_shoot_button_pressed();
        } else { // Stop intake vertical wheels
          intake.stopIntakeWheels();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.deploy(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
