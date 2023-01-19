package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    private double distance = 0.0;
    private Drivetrain drivetrain;
    private Pose2d initialPose = null;
    private double speed = 0.0;
    private double displacement = 0.0;
    public DriveDistance(Drivetrain drive, double dist, double s) {
        drivetrain = drive;
        distance = dist;
        speed = s;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialPose = drivetrain.getPose();
        // driveTime = distance/speed;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose();
        double x = pose.getX() - initialPose.getX();
        double y = pose.getY() - initialPose.getY();
        displacement = Math.sqrt((x*x) + (y*y));

        if (displacement - distance <=.1) {
            drivetrain.stop();
        } else {
            drivetrain.tankDriveVolts(speed, speed);
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
        return displacement - distance <=.1;
    }
}
