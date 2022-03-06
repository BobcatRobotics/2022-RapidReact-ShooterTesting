package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain;

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
    
    private Drivetrain dt;

    private JSONParser parser = new JSONParser();
    private NetworkTableEntry camJson;
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

    // UPDATE: 10:43 pm currently and I can't get slow zone to work, so I'm going to leave it incomplete for now - Arnav
    //private final double slowZone = 20;
    //private final double slowScalingFactor = 0.5;

    public AlignToNearestBall(Drivetrain drivetrain, String teamColor, double lv, double rv, double centerX) {
        color = teamColor;
        leftVolts = lv;
        rightVolts = rv;
        goalX = centerX;
        driveTime = time_allotted_per_miniTurn;

        dt = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // access network tables server
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // access pyVision table used in cargo_tracker.py
        table = inst.getTable("pyVision");
    }

    @Override
    public void execute() {
        // get jsonData entry made from cargo_tracker.py
        camJson = (String) table.getEntry("jsonData");
        // convert to jsonArray and then to object array
        JSONArray jsonArray = (JSONArray) parser.parse(camJson);
        Object[] jobArray = jsonArray.toArray();

        // get largest radius object from jobArray
        largestRadii = 0.0;

        for (Object job : jobArray) {
            job = (JSONObject) job;
            if (job.get("radius") > largestRadii && job.get("color") == color) {
                largestRadiiBall = job;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        dt.stop();
        dt.brake();
    }
    
    @Override
    public boolean isFinished() {
        camJson = (String) table.getEntry("jsonData");
        JSONArray jsonArray = (JSONArray) parser.parse(camJson);
        Object[] jobArray = jsonArray.toArray();
        
        JSONObject updated;

        // reset timer
        t.reset();

         // align till ball in approximate center of camera frame
         if (t.hasElapsed(driveTime)) {
            drivetrain.stop();
            t.reset();
        } else {
            t.start();
            drivetrain.tankDriveVolts(leftVolts, rightVolts);
        }

        // find ball with similar radius as largestRadii Ball
        for (Object job : jobArray) {
            job = (JSONObject) job;
            if (largestRadiiBall.get("radius") - ballRadiiError <= job.get("radius") <= largestRadiiBall.get("radius") + ballRadiiError) {
                updated = job;
                break;
            }
        }

        largestRadiiBall = updated;

        // end command if ball aligned pretty close to center
        if (goalX - finalRange <= largestRadiiBall.get("centerX") <= goalX + finalRange) {
            return true;
        }

        return false;
    }
}
