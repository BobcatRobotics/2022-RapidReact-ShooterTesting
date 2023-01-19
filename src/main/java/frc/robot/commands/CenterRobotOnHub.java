package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.controller.PIDController;

public class CenterRobotOnHub extends CommandBase {

    private final Drivetrain drivetrain;
    private Joystick gamepad;
    // private NetworkTable limelightTable;
    private Limelight limelight;
  
    private PIDController pidController;
    private Timer timeout;
    
    public CenterRobotOnHub(Drivetrain dt, Joystick gp, Limelight lm) {
        drivetrain = dt;
        gamepad = gp;
        limelight = lm;
        timeout = new Timer();
        if (!limelight.isInitialized()) {
            limelight.initializeLimeLight();
        }
        // SmartDashboard.putBoolean("d-pad left", gamepad.getPOV() == Constants.D_Pad_Left);
        // SmartDashboard.putNumber("LimeLight drive percent", 0.0);
        // RioLogger.errorLog("CenterRobotOnHub started");
        // SmartDashboard.putBoolean("shouldStop", false);
        // SmartDashboard.putNumber("CROH kP", .72);
        // SmartDashboard.putNumber("CROH kI", 0.2);
        // SmartDashboard.putNumber("CROH kD", 0.03);
        pidController = new PIDController(0.9, .07, 0);
        addRequirements(limelight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeout.reset();
        timeout.start();
        // RioLogger.errorLog("CenterRobotOnHub started");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // pidController.setP(SmartDashboard.getNumber("CROH kP", 0.72));
        // pidController.setI(SmartDashboard.getNumber("CROH kI", 0.2));
        // pidController.setD(SmartDashboard.getNumber("CROH kD", 0.03));
        // limelight.updateFrame();
        // Run command when joystick trigger is pressed
        // SmartDashboard.putBoolean("d-pad left", gamepad.getPOV() == Constants.D_Pad_Left);
        
        if (gamepad.getPOV() == Constants.D_Pad_Left || SmartDashboard.getBoolean("AutoMode", true) ) {
            // limelight.updateFrame();
            // Limelight has not found any targets
            limelight.turnOnLED();
            if (!limelight.hasTargets()) {
                // Stop drivetrain
                drivetrain.stop();
            }
            // Limelight has found a target
            else {
                // Get horizontal offset from crosshair to target
                double tx = limelight.x();
                // Normalize between 0 and 1 by dividing by max degree offset: 29.8 deg
                tx /= 14.9;
                // Turn
                // if (Math.abs(tx) < 0.1) { // Threshold to stop turning
                //     SmartDashboard.putBoolean("shouldStop", true);
                //     // m_drivetrain.stop();
                // } else {
                //     SmartDashboard.putBoolean("shouldStop", false);
                // }
                // Give at least a little bit of turn just in case
                // if (tx > 0) tx += 0.12;
                // else tx -= 0.12;
                // Make sure not over 1 or below -1
                if (tx > 0.5) tx = 0.5;
                else if (tx < -0.5) tx = -0.5;
                // Turn
                // SmartDashboard.putNumber("LimeLight drive percent", tx);
                double d = pidController.calculate(tx, 0);
                if (Math.abs(d) < .05) {
                    d = .125 * Math.signum(d);
                }
                drivetrain.drive(d, -d, true);
                // }
            }
        } else {
            if (gamepad.getPOV() != Constants.D_Pad_Right) {
                limelight.turnOffLED();
            }
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
        // System.out.println("WHOA WHOA");
        drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(SmartDashboard.getBoolean("AutoMode", true)) {
            // double tx = limelight.x();
            // // Normalize between 0 and 1 by dividing by max degree offset: 29.8 deg
            // tx /= 7.25;
            // if (Math.abs(tx) <.1)
            //     return true;
            return timeout.hasElapsed(2);
        }
        return false;
        // return limelightTable.getEntry("tv") && Math.abs(limelightTable.getEntry("tx").getDouble(defaultValue));
    }

}
