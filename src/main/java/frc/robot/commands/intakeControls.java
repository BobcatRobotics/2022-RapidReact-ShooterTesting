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
  private Shooter shooter;
  private Climber climber;
  private Joystick gp;

  public intakeControls(Intake in, Joystick gp, Shooter shoot, Climber climber) {
    this.intake = in;
    this.gp = gp;
    this.shooter = shoot;
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

    boolean feedReady = shooter.getBallReadyToFeed();

    // Gamepad X button -> intake up
    if (gp.getRawButton(Constants.X_Button) && !climber.isClimberMode()) {
      if (intake.isDeployed()) {
        System.out.println("X button - intake up");
        intake.deploy(false);
      }
    }
    // Gamepad Y button -> intake down
    else if (gp.getRawButton(Constants.Y_Button) && !climber.isClimberMode()) {
      if (!intake.isDeployed()) {
        System.out.println("Y button - intake down");
        intake.deploy(true);
      }
    }

    // Gamepad B button -> intake + vertical wheels out
    if (gp.getRawButton(Constants.B_Button)) {
      System.out.println("B button");
      intake.feedOut();
    } else {

      // Gamepad right trigger -> vertical wheels in
      if (gp.getRawButton(Constants.Right_Trigger_Button)) {
        System.out.println("Right trigger button");
        intake.runIntakeWheelsIn(true);
      }
      // Stop vertical wheels if nothing
      else {
        intake.stopIntakeWheels();
      }


      // Gamepad right bumper button -> intake bar in
      if (gp.getRawButton(Constants.Right_Bumper_Button)) {
        // WORKING
        System.out.println("Right bumper button");
        intake.runIntakeBarIn(true);
      }
      // Gamepad D-pad down -> intake out
      else if (gp.getPOV() == Constants.D_Pad_Down) {
        System.out.println("D-pad down button");
        intake.runIntakeBarOut(true);
      }
      // Stop intake bar if nothing
      else {
        intake.stopIntakeBar();
      }

    }

    // if (gp.getRawButton(Constants.Left_Bumper_Button)) { 
    //   intake.feedOut();
    // } else if (gp.getRawButton(Constants.Right_Bumper_Button)) {
    //   if (feedReady){
    //     intake.feedIn();
    //   } else {
    //     intake.runIntakeBarIn(true);
    //   }
    // } else {
    //   intake.stopIntake();
    // }

    // //toggle the solenoid 
    // if (gp.getRawButton(Constants.X_Button)) {
    //   intake.toggleDeploy();
    // }

    // //extra feed conntrols
    // if (gp.getRawButton(Constants.B_Button)) {
    //   intake.runIntakeWheelsIn(true);
    //   intake.runIntakeBarIn(true);
    // } else if(gp.getRawButton(Constants.A_Button)) {
    //   intake.runIntakeWheelsOut(true);
    //   intake.runIntakeBarOut(true);
    // } else {
    //   intake.stopIntake();
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.deploy(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
