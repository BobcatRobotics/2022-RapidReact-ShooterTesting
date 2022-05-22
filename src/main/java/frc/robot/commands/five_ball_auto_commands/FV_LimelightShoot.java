package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.RioLogger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class FV_LimelightShoot extends CommandBase {

    private Limelight limelight;
    private Shooter shooter;
    private Intake intake;
    private double shootingDist;
    private Timer timeoutTimer;
    private double timeoutSec;
    private Timer tofTimer;
    private boolean useLimelight;
    private Timer initTimer;
    private double initWaitPeriod;
    
    public FV_LimelightShoot(Limelight _limelight, Shooter _shooter, Intake _intake, double _timeoutSec, boolean _useLimelight, double _initWait) {
        limelight = _limelight;
        shooter = _shooter;
        intake = _intake;
        timeoutSec = _timeoutSec;
        useLimelight = _useLimelight;
        timeoutTimer = new Timer();
        tofTimer = new Timer();
        initTimer = new Timer();
        initWaitPeriod = _initWait;
    }

    @Override
    public void initialize() {
        timeoutTimer.reset();
        timeoutTimer.start();
        tofTimer.reset();
        tofTimer.start();
        initTimer.reset();
        initTimer.start();
    }

    @Override
    public void execute() {
        limelight.turnOnLED();
        if (shooter.tofTriggered()) {
            tofTimer.stop();
            tofTimer.reset();
            tofTimer.start();
        }
        if (limelight.hasTargets() || !useLimelight) {
            if (useLimelight) {
                shootingDist = Math.round(4*(LimelightConstants.kLimelightHeight / Math.tan(limelight.y()*Math.PI/180 + LimelightConstants.kLimelightMountAngle)))/4.0;
            } else {
                shootingDist = Constants.ShooterConstants.UPPER_HUB_KEY;
            }
            // double[] speeds = distToRPM(shootingDist);
            // Shooter get to speed and shoot at velocity
            shooter.setShootingModeKey(shootingDist);
            // System.out.printf("FV: Will shoot at %s RPM based on %s meters away\n", speeds[0], speeds[1]);
            // Ready to shoot
            shooter.getToSpeed();
            if (shooter.atSpeed() && initTimer.hasElapsed(initWaitPeriod)) {
                intake.feedIn();;
                shooter.feed();
            } else {
                shooter.stopFeeding();
                intake.stopIntake();
            }
        }
    }

    public double[] distToRPM(double dist) {
        if (dist < 1.0) dist = 1.0;
        if (dist > 8.0) dist = 8.0;
        return ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(dist);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stopIntake();
        if (interrupted) {
            RioLogger.errorLog("FV_LIMELIGHT_SHOOT: Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        if (timeoutTimer.hasElapsed(timeoutSec) || tofTimer.hasElapsed(3)) {
            return true;
        }
        return false;
    }

}
