package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.RioLogger;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class LimelightShootDist extends CommandBase {

    private Limelight limelight;
    private Joystick rightStick;
    private Shooter shooter;
    private final double kLimelightMountAngle = 40.0 * Math.PI/180; // rad
    private final double kLimelightHeight = 2.64 - 0.532; // hub height - limelight mount height, meters
    private double shootingDist;
    
    public LimelightShootDist(Limelight _limelight, Joystick _rightStick, Shooter _shooter) {
        limelight = _limelight;
        rightStick = _rightStick;
        shooter = _shooter;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (rightStick.getTrigger()) {
            limelight.turnOnLED();
            if (limelight.hasTargets()) {
                shootingDist = Math.round(2*(kLimelightHeight / Math.tan(limelight.y()*Math.PI/180 + kLimelightMountAngle)))/2.0;
                double[] speeds = distToRPM(shootingDist);
                // Shooter get to speed and shoot at velocity
                System.out.printf("Will shoot at %s RPM based on %s meters away\n", speeds[0], speeds[1]);
                // Ready to shoot
                if (rightStick.getTop()) {
                    // shooter.setHighMode(true);
                    shooter.getToSpeed();
                    if (shooter.atSpeed()) {
                        shooter.feed();
                    }
                }
                // Once button is no longer being pressed, run below only once
                else if (rightStick.getTopReleased()) {
                    shooter.stop();
                }
            }
        }
        // Once button is no longer being pressed, run below only once
        else if (rightStick.getTriggerReleased()) {
            limelight.turnOffLED();
        }
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

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            RioLogger.errorLog("LIMELIGHT_SHOOT_VAR_DIST: Interrupted");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void updateShuffleboardForSpeedTests() {

    }

}
