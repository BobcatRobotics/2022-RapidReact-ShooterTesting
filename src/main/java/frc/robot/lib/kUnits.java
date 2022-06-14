package frc.robot.lib;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RouteFinderConstants;

public class kUnits {
    public static double NU2Meters(double rawUnits) {
        // rawUnits is in natural units per 100 ms
        return rawUnits * 10 / DrivetrainConstants.encoderCPR / RouteFinderConstants.gearRatio * RouteFinderConstants.wheelCircumferenceMeters * 2;
    }

    public static double meters2NU(double meters) {
        return meters / 10 * DrivetrainConstants.encoderCPR * RouteFinderConstants.gearRatio / RouteFinderConstants.wheelCircumferenceMeters / 2;
    }
}
