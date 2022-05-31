package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.lib.kUnits;

public class Drivetrain extends SubsystemBase {

    private TalonFX ltMotor;
    private TalonFX lmMotor;
    private TalonFX lbMotor;
    private TalonFX rtMotor;
    private TalonFX rmMotor;
    private TalonFX rbMotor;
    private boolean isBreak = true;

    private final NavxGyro gyro = new NavxGyro(SPI.Port.kMXP); // The gyro sensor

    private final DifferentialDriveOdometry odometry; // Odometry class for tracking robot pose

    public Drivetrain() {
        configureMotor(ltMotor, DrivetrainConstants.LTMotorPort, true);
        configureMotor(lmMotor, DrivetrainConstants.LMMotorPort, true);
        configureMotor(lbMotor, DrivetrainConstants.LLMotorPort, true);
        configureMotor(rtMotor, DrivetrainConstants.RTMotorPort, false);
        configureMotor(rmMotor, DrivetrainConstants.RMMotorPort, false);
        configureMotor(rbMotor, DrivetrainConstants.RLMotorPort, false);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
        resetEncoders();
        zeroHeading();
        isBreak = true;
        lowerCANBusUtilization();
    }

    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(),
            ltMotor.getSelectedSensorPosition() / (2 * DrivetrainConstants.gearRatio * DrivetrainConstants.kSensorUnitsPerRotation),
            rtMotor.getSelectedSensorPosition() / (2 * DrivetrainConstants.gearRatio * DrivetrainConstants.kSensorUnitsPerRotation)
        );
    }

    public void configureMotor(TalonFX motor, int port, boolean setInverted) {
        motor = new TalonFX(port);
        motor.configFactoryDefault();
        // motor.config_kF(0, 0.1);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(setInverted);
        // motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    }

    public void lowerCANBusUtilization() {
        for (TalonFX talon : new TalonFX[] { ltMotor, lmMotor, lbMotor, rtMotor, rmMotor, rbMotor }) {
            // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION
            // FREQUENCIES FOR UNUSED MOTOR OUTPUT
            talon.setStatusFramePeriod(StatusFrame.Status_1_General, 20, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255, Constants.kTimeoutMs);
            talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255, Constants.kTimeoutMs);
        }
    }

    public void setLeftVelocityPercent(double leftPercent) {
        ltMotor.set(ControlMode.PercentOutput, leftPercent);
        lmMotor.set(ControlMode.PercentOutput, leftPercent);
        lbMotor.set(ControlMode.PercentOutput, leftPercent);
    }

    public void setRightVelocityPercent(double rightPercent) {
        rtMotor.set(ControlMode.PercentOutput, rightPercent);
        rmMotor.set(ControlMode.PercentOutput, rightPercent);
        rbMotor.set(ControlMode.PercentOutput, rightPercent);
    }

    public void setVelocityMetersPerSecond(double leftMPS, double rightMPS) {
        double leftRawUnitsPer100 = kUnits.meters2NU(leftMPS);
        double rightRawUnitsPer100 = kUnits.meters2NU(rightMPS);
        ltMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
        lmMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
        lbMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
        rtMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
        rmMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
        rbMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
    }

    public void setVolts(double leftVolts, double rightVolts) {
        setLeftVelocityPercent(leftVolts / 12);
        setRightVelocityPercent(rightVolts / 12);
    }

    public void coast() {
        isBreak = false;
        ltMotor.setNeutralMode(NeutralMode.Coast);
        lmMotor.setNeutralMode(NeutralMode.Coast);
        lbMotor.setNeutralMode(NeutralMode.Coast);
        rtMotor.setNeutralMode(NeutralMode.Coast);
        rmMotor.setNeutralMode(NeutralMode.Coast);
        rbMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void brake() {
        isBreak = true;
        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public boolean isBrake() {
        return isBreak;
    }

    public void stopRightMotor() {
        setRightVelocityPercent(0);
    }

    public void stopLeftMotor() {
        setLeftVelocityPercent(0);
    }

    /**
     * Drive with custom values
     * 
     * @param rightP Right motor power
     * @param leftP  Left motor power
     */
    public void drive(double rightP, double leftP, boolean shouldSquare) {
        if (shouldSquare) {
            rightP *= rightP * Math.signum(rightP);
            leftP *= leftP * Math.signum(leftP);
        }
        setRightVelocityPercent(rightP);
        setLeftVelocityPercent(leftP);
    }

    /**
     * Stops the drive train
     */
    public void stop() {
        stopRightMotor();
        stopLeftMotor();
    }

    /**
     * Returns the currently-estimated pose of the robot
     * 
     * @return The pose. (position on the field)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * 
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            kUnits.NU2Meters(ltMotor.getSelectedSensorVelocity())/2,
            kUnits.NU2Meters(rtMotor.getSelectedSensorVelocity())/2
        );
    }

    /**
     * Resets the odometry to a default pose Pose2d(0, 0, new Rotation2d(0))
     * Resets the encoders, also automatically resets heading
     */
    public void resetOdometry() {
        // Consider running zeroHeading() here or in the other
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Resets the odometry to the specified pose
     * Resets the encoders, also automatically resets heading
     * 
     * @param pose The pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        // Consider running zeroHeading() here or in the other
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Resets the drive encoders to currently read a position of 0
     */
    public void resetEncoders() {
        ltMotor.setSelectedSensorPosition(0.0);
        lmMotor.setSelectedSensorPosition(0.0);
        lbMotor.setSelectedSensorPosition(0.0);
        rtMotor.setSelectedSensorPosition(0.0);
        rmMotor.setSelectedSensorPosition(0.0);
        rbMotor.setSelectedSensorPosition(0.0);
    }

    /**
     * Zeroes the heading of the robot
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot
     * 
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate();
    }
}