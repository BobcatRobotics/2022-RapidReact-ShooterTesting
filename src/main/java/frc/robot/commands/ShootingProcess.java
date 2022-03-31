// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootingProcess extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private final Intake intake;
  private final Joystick gamepad;
  private final Climber climber;
  private final Limelight limelight;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ShootingProcess(Shooter shooter, Intake intake, Joystick gamepad, Climber climber, Limelight limelight) {
    this.shooter = shooter;
    this.intake = intake;
    this.gamepad = gamepad;
    this.climber = climber;
    this.limelight = limelight;
    SmartDashboard.putNumber("Limelight dist (m)", 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  public double[] distToRPM(double dist) {
    // [PR] Linear function temporarily; earlier tests yielded that from
    // ~3.4544 meters away from the center of the hub, 4000 RPM was a
    // good shooting RPM.
    // return dist * 4000 / 3.4544;
    if (dist < 1.0) dist = 1.0;
    if (dist > 8.0) dist = 8.0;
    return ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(dist);
    // NOTE: The relationship is likely quadratic. However, to construct the
    // appropriate function, we need to conduct shooting tests at different
    // distances. We should figure out when to do this.
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // Gamepad D-pad right -> shooter solenoid up
    // if (gamepad.getPOV() == Constants.D_Pad_Right && !climber.isClimberMode()) {
    //   if (!shooter.isShooterSolenoidExtended()) {
    //     // System.out.println("D-pad right - shooter solenoid up");
    //     shooter.setShooterSolenoidExtended(true);
    //   }
    // }
    // // Gamepad D-pad left -> shooter solenoid down
    // else if (gamepad.getPOV() == Constants.D_Pad_Left && !climber.isClimberMode()) {
    //   if (shooter.isShooterSolenoidExtended()) {
    //     // System.out.println("D-pad left - shooter solenoid down");
    //     shooter.setShooterSolenoidExtended(false);
    //   }
    // }

    // Gamepad D-pad right -> run shooter Limelight-based speed
    if (gamepad.getPOV() == Constants.D_Pad_Right && !climber.isClimberMode()) {
      if (limelight.hasTargets()) {
          shooter.setShootingModeKey(Math.round(2*(LimelightConstants.kLimelightHeight / Math.tan(limelight.y()*Math.PI/180 + LimelightConstants.kLimelightMountAngle)))/2.0);
          // Shooter get to speed and shoot at velocity
          // System.out.printf("Will shoot at %s RPM based on %s meters away\n", speeds[0], speeds[1]);
          // Ready to shoot
          shooter.setRunning(true);
          shooter.getToSpeed();
          if (shooter.atSpeed()) {
              shooter.feed();
          }
      } else if (!intake.isIntakeRunningIn()) {
        shooter.stopFeeding();
      }
    }
    // Gamepad left bumper button -> run shooter high speed
    else if (gamepad.getRawButton(Constants.Left_Bumper_Button)) {
      // shooter.setHighMode(true);
      shooter.setShootingModeKey(ShooterConstants.UPPER_HUB_KEY);
      shooter.setRunning(true);
      shooter.getToSpeed();
    }
    // Gamepad D pad up -> run shooter low speed
    else if (gamepad.getPOV() == Constants.D_Pad_Up) {
      shooter.setShootingModeKey(ShooterConstants.LOWER_HUB_KEY);
      shooter.setRunning(true);
      shooter.getToSpeed();
    }
    // Stop shooter
    else {
      shooter.stopShooter();
      shooter.setRunning(false);
    }


    // Gamepad left trigger -> run tower in
    if (gamepad.getRawButton(Constants.Left_Trigger_Button)) {
      if (shooter.atSpeed()) {
        shooter.feed();
      }
    } else if (gamepad.getRawButton(Constants.Right_Joystick_Pressed)) {
      shooter.reverseFeed();
    } else if (!intake.isIntakeRunningIn()) {
      shooter.stopFeeding();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }
}
