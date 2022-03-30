package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.lib.RioLogger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;
// import frc.robot.Constants.RobotContainerConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CenterRobotOnHub extends CommandBase {

    private final Drivetrain drivetrain;
    private Joystick rightStick;
    // private NetworkTable limelightTable;
    private Limelight limelight;
    private boolean ledIsOn = false;
    private PIDController pidController;
    
    public CenterRobotOnHub(Drivetrain dt, Joystick rightJ, Limelight lm) {
        drivetrain = dt;
        rightStick = rightJ;
        limelight = lm;
        ledIsOn = false;
        if (!limelight.isInitialized()) {
            limelight.initializeLimeLight();
        }
        SmartDashboard.putNumber("LimeLight drive percent", 0.0);
        RioLogger.errorLog("CenterRobotOnHub started");
        SmartDashboard.putBoolean("shouldStop", false);
        SmartDashboard.putNumber("CROH kP", 1);
        SmartDashboard.putNumber("CROH kI", 0);
        SmartDashboard.putNumber("CROH kD", 0);
        pidController = new PIDController(1, 0, 0);
        addRequirements(limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        RioLogger.errorLog("CenterRobotOnHub started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pidController.setP(SmartDashboard.getNumber("CROH kP", 1));
        pidController.setI(SmartDashboard.getNumber("CROH kI", 0));
        pidController.setD(SmartDashboard.getNumber("CROH kD", 0));
        // limelight.updateFrame();
        // Run command when joystick trigger is pressed
        if (rightStick.getTrigger()) {
            // limelight.updateFrame();
            // Limelight has not found any targets
            if (!limelight.hasTargets()) {
                // Stop drivetrain
                drivetrain.stop();
            }
            // Limelight has found a target
            else {
                // Get horizontal offset from crosshair to target
                double tx = limelight.x();
                // Normalize between 0 and 1 by dividing by max degree offset: 29.8 deg
                tx /= 29.8;
                // Turn
                if (Math.abs(tx) < 0.01) { // Threshold to stop turning
                    SmartDashboard.putBoolean("shouldStop", true);
                    // m_drivetrain.stop();
                } else {
                    SmartDashboard.putBoolean("shouldStop", false);
                }
                // Give at least a little bit of turn just in case
                // if (tx > 0) tx += 0.12;
                // else tx -= 0.12;
                // Make sure not over 1 or below -1
                if (tx > 1) tx = 1;
                else if (tx < -1) tx = -1;
                // Turn
                SmartDashboard.putNumber("LimeLight drive percent", tx);
                double d = pidController.calculate(tx, 0);
                drivetrain.drive(d, d);
                // }
            }
        } else {
            // Turn off limelight light
            // if (ledIsOn) {
            //     m_limelight.turnOffLED();
            //     ledIsOn = false;
            // }
        }
        // if (rightJoystick.getRawButtonReleased(Constants.D_Pad_Left)) {
        //     if (ledIsOn) {
        //         limelight.turnOffLED();
        //         ledIsOn = false;
        //     } else {
        //         limelight.turnOnLED();
        //         ledIsOn = true;
        //     }
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("WHOA WHOA");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        // return limelightTable.getEntry("tv") && Math.abs(limelightTable.getEntry("tx").getDouble(defaultValue));
    }

}
