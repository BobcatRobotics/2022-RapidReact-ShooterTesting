package frc.robot.lib;

import frc.robot.Constants.DrivetrainConstants;

public class kUnits {
    public static double NU2Meters(double rawUnits) {
        // rawUnits is in natural units per 100 ms
        return rawUnits * 10 / DrivetrainConstants.kSensorUnitsPerRotation / DrivetrainConstants.gearRatio * DrivetrainConstants.wheelCircumferenceMeters * 2;
    }

    public static double meters2NU(double meters) {
        return meters / 10 * DrivetrainConstants.kSensorUnitsPerRotation * DrivetrainConstants.gearRatio / DrivetrainConstants.wheelCircumferenceMeters / 2;
    }
}
