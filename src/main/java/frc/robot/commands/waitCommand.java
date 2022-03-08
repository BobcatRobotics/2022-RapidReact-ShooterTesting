package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class waitCommand extends CommandBase {
    private Timer timer = new Timer();
    private double time;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    public waitCommand(double _time) {
        time = _time;
    }

    /**
     * Instantiation - will turn off all subsystems to start
     */
    public waitCommand(double _time, Drivetrain dt, Intake it, Shooter sh) {
        time = _time;
        drivetrain = dt;
        intake = it;
        shooter = sh;
        addRequirements(dt, it, sh);
    }

    @Override
    public void initialize() {
        if (drivetrain != null) drivetrain.stop();
        if (intake != null) {
            intake.deploy(false);
            intake.stopIntake();
        }
        if (shooter != null) shooter.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}
