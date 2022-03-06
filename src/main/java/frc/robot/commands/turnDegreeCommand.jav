package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class driveStraightCommand extends CommandBase {
    private double distance;
    private Drivetrain drivetrain;
    private double turnDegree = 0.0;
    private double heading = 0.0;
    private double goalHeading = 0.0;
    private double speed = 0.0;


    private boolean headingInit = false;


    public driveStraightCommand(Drivetrain drive, double d) {
        drivetrain = drive;
        turnDegree = d;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void beforeStarting() {
        heading = Rotation2d.fromDegrees(drivetrain.getHeading());
        goalHeading = heading + turnDegree;
        headingInit = true;
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        heading = Rotation2d.fromDegrees(drivetrain.getHeading());
        if(!headingInit) {
            goalHeading = heading + turnDegree;
            headingInit = true;
        }

        double degrees = (goalHeading - heading) % 360.0;
        
        if ((degrees <= 0.5 && degrees >= 0.0) || (degrees >= -0.5 && degrees <= 0.0) ) {
            degrees = 0.0;
        }


        if (Math.signum(degrees) == 1.0) {
            // sign is positive so turn right
            drivetrain.tankDriveVolts(3,-3);

        } else if (Math.signum(degrees) == -1.0) {
            drivetrain.tankDriveVolts(-3,3);

        } else {
            drivetrain.stop();

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
