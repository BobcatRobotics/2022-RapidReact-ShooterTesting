package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FV_PID_Turn extends CommandBase {
    private double distance, turnDegree = 0.0, heading=0.0, goalHeading=0.0, speed=0.0;
    private Drivetrain drivetrain;
    private boolean headingInit = false;
    private double Degrees = 180;
    private PIDController pidController;

    public FV_PID_Turn(Drivetrain dt, double _turnDegree) {
        drivetrain = dt;
        turnDegree = _turnDegree;
        drivetrain.coast();
        pidController = new PIDController(0.75, 0.2, 0);
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

        double drivePower = pidController.calculate(degrees/180, 0);

        drivetrain.drive(drivePower, -drivePower, false);
        // if (Math.signum(degrees) == 1.0) {
        //     // drivetrain.tankDriveVolts(-2,2);
        // } else if (Math.signum(degrees) == -1.0) {
        //     // drivetrain.tankDriveVolts(2,-2);
        // } else {
        //     drivetrain.stop();
        // }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.brake();
    }

    @Override
    public boolean isFinished() {
        // return t.hasElapsed(driveTime);
        return Degrees == 0.0;
    }
}
