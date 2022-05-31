package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FV_DriveTime extends CommandBase {
    private Drivetrain drivetrain;
    private Timer t = new Timer();
    private double driveTime = 0.0;
    private double leftVolts, rightVolts;
    public FV_DriveTime(Drivetrain drive, double lv, double rv, double time_allotted) {
        drivetrain = drive;
        driveTime = time_allotted;
        leftVolts = lv;
        rightVolts = rv;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        t.reset();
        t.start();
        // driveTime = distance/speed;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (t.hasElapsed(driveTime)) {
            drivetrain.stop();
        } else {
            drivetrain.setVolts(leftVolts, rightVolts);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.brake();
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return t.hasElapsed(driveTime);
    }
}
