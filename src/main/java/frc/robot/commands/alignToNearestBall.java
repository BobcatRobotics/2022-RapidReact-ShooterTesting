// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drivetrain;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
// import org.json.simple.parser.JSONParser;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

// import edu.wpi.first.wpilibj.Timer;

// /*
// So far, AlignToNearestBall makes small, tiny 0.1 (abstract time to be changed w/ testing) second increments at a certain speed,
// turning until the center x-value of the ball with the largest radius is reasonably within the goal x-value which we would like
// it to be at. It is not being used anywhere and requires a functional cargo_tracker.py to work. As I commented below, I could not
// figure out how to implement the slow zone, so that is not present in this implementation - Arnav
// */

// public class AlignToNearestBall extends CommandBase {
    
    private Drivetrain drivetrain;
    private double angle = -1000;
    private RobotContainer m_rContainer;

    // UPDATE: 10:43 pm currently and I can't get slow zone to work, so I'm going to leave it incomplete for now - Arnav
    //private final double slowZone = 20;
    //private final double slowScalingFactor = 0.5;

    public alignToNearestBall(Drivetrain dt, RobotContainer r) {
        drivetrain = dt;
        m_rContainer = r;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Ball closestBall = m_rContainer.getClosestBall();

        if (closestBall != null) {
            angle = closestBall.getAngle();
            // 27 = camera fov for normalization
            // .4 = scaling factor so it doesn't turn too fast
            double drivePower = (angle/27)*.4;
            drivetrain.drive(drivePower , drivePower);
        } else {
            drivetrain.drive(2.5,2.5);
        }

        if (angle <=.2) {
            angle = 0.0;
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.brake();
    }
    
    @Override
    public boolean isFinished() {
        if(angle == 0.0) {
            return true;
        } else {
            return false;
        }
    }
}
