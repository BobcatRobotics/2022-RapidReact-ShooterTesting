package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class turnDegreeCommand extends CommandBase {
    private double turnDegree = 0.0, heading=0.0, goalHeading=0.0;
    private Drivetrain drivetrain;
    private boolean headingInit = false;
    private double Degrees = 180;

    public turnDegreeCommand(Drivetrain dt, double d) {
        drivetrain = dt;
        turnDegree = d;
        drivetrain.coast();
        addRequirements(drivetrain);
    }

    // @Override
    // public void beforeStarting() {
    //     heading = Rotation2d.fromDegrees(drivetrain.getHeading()).getDegrees();
    //     goalHeading = heading + turnDegree;
    //     headingInit = true;
    // }

    @Override
    public void execute() {
        heading = Rotation2d.fromDegrees(drivetrain.getHeading()).getDegrees();
        if (!headingInit) {
            goalHeading = heading + turnDegree;
            headingInit = true;
        }

        double degrees = (goalHeading - heading) % 360.0;

        if ((degrees <= 0.3 && degrees >= 0.0) || (degrees >= -0.3 && degrees <= 0.0)) {
            degrees = 0.0;
        }

        Degrees = degrees;

        if (Math.signum(degrees) == 1.0) {
            drivetrain.tankDriveVolts(-2,2);
        } else if (Math.signum(degrees) == -1.0) {
            drivetrain.tankDriveVolts(2,-2);
        } else {
            drivetrain.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IT'S DONE FELLAS LETS GOOOOOOOO");
        drivetrain.stop();
        drivetrain.brake();
    }

    @Override
    public boolean isFinished() {
        // return t.hasElapsed(driveTime);
        return Degrees == 0.0;
    }
}
