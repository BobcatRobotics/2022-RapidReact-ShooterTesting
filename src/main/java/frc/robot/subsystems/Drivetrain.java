package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.lib.kUnits;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {

    private TalonFX ltMotor, lmMotor, lbMotor;
    private TalonFX rtMotor, rmMotor, rbMotor;

    private final NavxGyro gyro = new NavxGyro(SPI.Port.kMXP);

    private final DifferentialDriveOdometry odometry;

    public Drivetrain() {
        ltMotor = new TalonFX(0);
        lmMotor = new TalonFX(1);
        lbMotor = new TalonFX(2);
        rtMotor = new TalonFX(3);
        rmMotor = new TalonFX(4);
        rbMotor = new TalonFX(5);

        configDefault(ltMotor, true);
        configDefault(lmMotor, true);
        configDefault(lbMotor, true);
        configDefault(rtMotor, false);
        configDefault(rmMotor, false);
        configDefault(rbMotor, false);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        resetEncoders();
        zeroHeading();
        gyro.calibrate();
        resetOdometry();
    }

    @Override
    public void periodic() {
        //odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance()); // Update the odometry in the periodic block (gets location on field)
        //periodic called ever .02s
        // odometry.update(Rotation2d.fromDegrees(getHeading()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
        odometry.update(gyro.getRotation2d(), ltMotor.getSelectedSensorPosition()/42065, rtMotor.getSelectedSensorPosition()/42065);
    }

    public void configDefault(TalonFX talonFXMotor, boolean shouldInvert) {
        talonFXMotor.configFactoryDefault();
        talonFXMotor.config_kF(0, 0.1);
        talonFXMotor.setInverted(shouldInvert);
    }

    public void setVolts(double leftVolts, double rightVolts){
        ltMotor.set(ControlMode.PercentOutput, -leftVolts / 24);
        lmMotor.set(ControlMode.PercentOutput, -leftVolts / 24);
        lbMotor.set(ControlMode.PercentOutput, -leftVolts / 24);
        rtMotor.set(ControlMode.PercentOutput, -rightVolts / 24);
        rmMotor.set(ControlMode.PercentOutput, -rightVolts / 24);
        rbMotor.set(ControlMode.PercentOutput, -rightVolts / 24);
    }

    public void setPercent(double leftP, double rightP){
        ltMotor.set(ControlMode.PercentOutput, leftP);
        lmMotor.set(ControlMode.PercentOutput, leftP);
        lbMotor.set(ControlMode.PercentOutput, leftP);
        rtMotor.set(ControlMode.PercentOutput, rightP);
        rmMotor.set(ControlMode.PercentOutput, rightP);
        rbMotor.set(ControlMode.PercentOutput, rightP);
    }

    public void setVelocityRaw(double leftRawUnitsPer100, double rightRawUnitsPer100) {
        setLeftVelocity(leftRawUnitsPer100);
        setRightVelocity(leftRawUnitsPer100);
    }

    public void setVelocityMetersPerSecond(double leftMPS, double rightMPS) {
        setLeftVelocity(kUnits.meters2NU(leftMPS));
        setRightVelocity(kUnits.meters2NU(rightMPS));
    }

    public void setLeftVelocity(double leftRawUnitsPer100) {
        ltMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
        lmMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
        lbMotor.set(ControlMode.Velocity, leftRawUnitsPer100);
    }
    
    public void setRightVelocity(double rightRawUnitsPer100) {
        rtMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
        rmMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
        rbMotor.set(ControlMode.Velocity, rightRawUnitsPer100);
    }

    public void stop() {
        setLeftVelocity(0);
        setRightVelocity(0);
    }

    public void setBrakeMode() {
        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastMode() {
        ltMotor.setNeutralMode(NeutralMode.Coast);
        lmMotor.setNeutralMode(NeutralMode.Coast);
        lbMotor.setNeutralMode(NeutralMode.Coast);
        rtMotor.setNeutralMode(NeutralMode.Coast);
        rmMotor.setNeutralMode(NeutralMode.Coast);
        rbMotor.setNeutralMode(NeutralMode.Coast);
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
        // leftEncoder.reset();
        // rightEncoder.reset();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        //return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
        // double left = 10.0 * (ltMotor.getSelectedSensorVelocity() * 60)/ (Constants.DrivetrainConstants.gearRatio * Constants.DrivetrainConstants.encoderTicksPerRev);
        // double right = 10.0* (rtMotor.getSelectedSensorVelocity() * 60)/ (Constants.DrivetrainConstants.gearRatio *  Constants.DrivetrainConstants.encoderTicksPerRev);
        double left = kUnits.NU2Meters(ltMotor.getSelectedSensorVelocity())/2;
        double right = kUnits.NU2Meters(rtMotor.getSelectedSensorVelocity())/2;
        return new DifferentialDriveWheelSpeeds(left,right);
                                            
    }

    /**
     * Zeroes the heading of the robot
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() ;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    public void resetOdometry() {
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void resetOdometry(Pose2d pose) {
        zeroHeading();
        resetEncoders();
        odometry.resetPosition(pose, pose.getRotation());
    }

    public void brake() {
        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void coast() {
        ltMotor.setNeutralMode(NeutralMode.Coast);
        lmMotor.setNeutralMode(NeutralMode.Coast);
        lbMotor.setNeutralMode(NeutralMode.Coast);
        rtMotor.setNeutralMode(NeutralMode.Coast);
        rmMotor.setNeutralMode(NeutralMode.Coast);
        rbMotor.setNeutralMode(NeutralMode.Coast);
    }
}
