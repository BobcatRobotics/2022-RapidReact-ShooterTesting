package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;

public class FV_LimelightHubAlign extends CommandBase {

    private final Drivetrain drivetrain;
    private Limelight limelight;
    private PIDController pidController;
    private double kScaleDown = 14.9;
    private Timer timeoutTimer;
    
    public FV_LimelightHubAlign(Drivetrain dt, Limelight lm) {
        drivetrain = dt;
        limelight = lm;
        if (!limelight.isInitialized()) {
            limelight.initializeLimeLight();
        }
        pidController = new PIDController(0.75, 0.2, 0);
        timeoutTimer = new Timer();
        addRequirements(limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Limelight has not found any targets
        limelight.turnOnLED();
        if (!limelight.hasTargets()) {
            // Spin until found
            drivetrain.drive(0.3, -0.3, false);
        }
        // Limelight has found a target
        else {
            timeoutTimer.stop();
            timeoutTimer.reset();
            timeoutTimer.start();
            // Get horizontal offset from crosshair to target
            double tx = limelight.x();
            // Normalize between 0 and 1 by dividing by max degree offset: 29.8 deg
            tx /= kScaleDown;
            // Make sure not over 1 or below -1
            if (tx > 1) tx = 1;
            else if (tx < -1) tx = -1;
            // Turn
            // SmartDashboard.putNumber("LimeLight drive percent", tx);
            double d = pidController.calculate(tx, 0);
            if (Math.abs(d) < .05) {
                d = .125 * Math.signum(d);
            }
            drivetrain.drive(d, -d, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("WHOA WHOA");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (timeoutTimer.hasElapsed(2)) return true;
        double tx = limelight.x();
        tx /= kScaleDown/2;
        return tx < 0.08;
    }

}
