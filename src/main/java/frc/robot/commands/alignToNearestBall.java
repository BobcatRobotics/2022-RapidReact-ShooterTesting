package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;

/*
So far, AlignToNearestBall makes small, tiny 0.1 (abstract time to be changed w/ testing) second increments at a certain speed,
turning until the center x-value of the ball with the largest radius is reasonably within the goal x-value which we would like
it to be at. It is not being used anywhere and requires a functional cargo_tracker.py to work. As I commented below, I could not
figure out how to implement the slow zone, so that is not present in this implementation - Arnav
*/

public class AlignToNearestBall extends CommandBase {
    
    private Drivetrain drivetrain;

    private JSONParser parser = new JSONParser();
    private String camJson;
    private NetworkTable table;
    private String color;

    private double distance;
    private double speed = 0.0;
    private double leftVolts, rightVolts;
    private double goalX;
    private double largestRadii;
    private JSONObject largestRadiiBall;
    private Timer t = new Timer();
    private double driveTime = 0.0;

    // arbitrary value of reasonable displacement from center of camera & displacement from center when motor speeds slow down & slowing down scaling factor && vision error on radius of same ball from very similar distances & time allotted per each tiny adjustment turn
    // TODO: set to appropriate values through testing
    private final double finalRange = 10;
    private final double ballRadiiError = 10;
    private final double time_allotted_per_miniTurn = 0.1;
    private double angle = -1000;

    // UPDATE: 10:43 pm currently and I can't get slow zone to work, so I'm going to leave it incomplete for now - Arnav
    //private final double slowZone = 20;
    //private final double slowScalingFactor = 0.5;

    public AlignToNearestBall(Drivetrain dt) {
        drivetrain = dt;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Ball closestBall = m_robotContainer.getClosestBall();

        if (closestBall != null) {
            angle = closestBall.get Angle();
            double drivePower = (angle/(Constants.BallCameraConstants.cameraFOV/2)/3);
            diveTrain.drive(drivePower,drivePower);
        } else {
            diveTrain.drive(3,3);
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        drivetrain.brake();
    }
    
    @Override
    public boolean isFinished() {
        if(angle = 0) {
            return true;
        }
    }
}
