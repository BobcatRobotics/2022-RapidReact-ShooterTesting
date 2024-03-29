/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RioLogger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight extends SubsystemBase {
    private boolean initialized = false;
    private NetworkTableEntry tTarget = null;
    private NetworkTableEntry tx = null;
    private NetworkTableEntry ty = null;
    private NetworkTableEntry ta = null;
    private NetworkTableEntry ta0 = null;
    private NetworkTableEntry ts0 = null;
    private NetworkTableEntry ta1 = null;
    private NetworkTableEntry ts1 = null;

    public Limelight() {
    }

    public void initializeLimeLight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        try {
            tTarget = table.getEntry("tv");

            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
            ta0 = table.getEntry("ta0");
            ts0 = table.getEntry("ts0");
            ta1 = table.getEntry("ta1");
            ts1 = table.getEntry("ts1");
        } catch (Exception e) {
            RioLogger.errorLog("Unable to initialize LimeLight. Error is " + e);
            return;
        }
        initialized = true;
    }

    public void updateFrame() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        try {
            tTarget = table.getEntry("tv");

            tx = table.getEntry("tx");
            ty = table.getEntry("ty");
            ta = table.getEntry("ta");
            ta0 = table.getEntry("ta0");
            ts0 = table.getEntry("ts0");
            ta1 = table.getEntry("ta1");
            ts1 = table.getEntry("ts1");
        } catch (Exception e) {
            RioLogger.errorLog("Unable to initialize LimeLight. Error is " + e);
            return;
        }
    }

    public NetworkTableEntry getEntry(String str) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(str);
    }

    public boolean isInitialized() {
        return this.initialized;
    }

    public boolean hasTargets() {
       boolean hits = false;
       if (isInitialized()) {
           hits = (getEntry("tv").getDouble(0.0) == 1.0);
       }
       return hits;
    }

    public double x() {
        double dx = 0.0;
        if (isInitialized()) {
            dx = getEntry("tx").getDouble(0.0);
        }
        return dx;
     }

     public double y() {
        double dy = 0.0;
        if (isInitialized()) {
            dy = getEntry("ty").getDouble(0.0);
        }
        return dy;
     }

     public double targetArea() {
        double dArea = 0.0;
        if (isInitialized()) {
            dArea = getEntry("ta").getDouble(0.0);
        }
        return dArea;
     }

     public double targetArea(int target) {
         double tgtArea = getEntry("ta0").getDouble(0.0);
         if (target == 1) {
             tgtArea = getEntry("ta1").getDouble(0.0);
         }
         return tgtArea;
     }

     public double targetSkew(int target) {
        double targetSkew = getEntry("ts0").getDouble(0.0);
        if (target == 1) {
            targetSkew = getEntry("ts1").getDouble(0.0);
        }
        return targetSkew;
    }

     public void turnOnLED() {
        lightLED(LimelightLED.ON);
     }

     public void turnOffLED() {
        lightLED(LimelightLED.OFF);
          
     }
     private void lightLED( LimelightLED value) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(value.ordinal());
        RioLogger.errorLog("Setting Limelight LEDs to "+ value.ordinal());
     }
}
