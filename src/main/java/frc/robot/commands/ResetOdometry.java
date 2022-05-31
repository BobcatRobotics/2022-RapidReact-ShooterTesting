package frc.robot.commands;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetOdometry extends CommandBase {

    private Drivetrain drivetrain;
    private Timer timeoutTimer;
    private double timeout;

    public ResetOdometry(Drivetrain dt, double timeout) {
        drivetrain = dt;
        timeoutTimer = new Timer();
    }
    
    @Override
    public void initialize() {
        drivetrain.zeroHeading();
        drivetrain.resetOdometry();
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timeoutTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return timeoutTimer.hasElapsed(timeout);
    }
}
