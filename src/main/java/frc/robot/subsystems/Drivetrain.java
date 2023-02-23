package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.INVERT_MOTOR;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.RouteFinderConstants.kTrackwidthMeters;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

   private WPI_TalonFX ltMotor;
   private WPI_TalonFX lmMotor;
   private WPI_TalonFX lbMotor;
   private WPI_TalonFX rtMotor;
   private WPI_TalonFX rmMotor;
   private WPI_TalonFX rbMotor;
   private boolean isBreak = true;

   private Limelight lime;


    // The motors on the left side of the drive.
    // private final SpeedControllerGroup leftMotors =
    /* new SpeedControllerGroup(
        new WPI_TalonFX(LTMotorPort),
        new WPI_TalonFX(LMMotorPort),
        new WPI_TalonFX(LLMotorPort)); */

    // The motors on the right side of the drive.
   // private final SpeedControllerGroup rightMotors = 
    /* new SpeedControllerGroup(
        new WPI_TalonFX(RTMotorPort),
        new WPI_TalonFX(RMMotorPort),
        new WPI_TalonFX(RLMotorPort))d;
 */
    // motor properties
    private double rightPower = 0.0;
    private double leftPower = 0.0;
    private boolean invertRight = false; // Whether or not to invert the right motor
    private boolean invertLeft = true; // Whether or not to invert the left motor
    private double voltageRegScaleFactor = 1.0; // TODO: Use this to adjust motor speed

    public double getVoltageRegScaleFactor() {
        return voltageRegScaleFactor;
    }

    //private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors); // The robot's drive


    private final AHRS gyro = new AHRS(SPI.Port.kMXP); // The gyro sensor

    private final DifferentialDriveOdometry odometry; // Odometry class for tracking robot pose
    private final DifferentialDrivePoseEstimator poseEstimator;
    /**
     * Method use to drive the robot
     */
    public Drivetrain() {
        //ChassisSpeeds speeds = new ChassisSpeeds(,,Math.PI); figure out what to do with this
        ltMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LTMotorPort);
        lmMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LMMotorPort);
        lbMotor = new WPI_TalonFX(Constants.DrivetrainConstants.LLMotorPort);
        rtMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RTMotorPort);
        rmMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RMMotorPort);
        rbMotor = new WPI_TalonFX(Constants.DrivetrainConstants.RLMotorPort);

        ltMotor.configFactoryDefault();
        lmMotor.configFactoryDefault();
        lbMotor.configFactoryDefault();
        rtMotor.configFactoryDefault();
        rmMotor.configFactoryDefault();
        rbMotor.configFactoryDefault();

        ltMotor.setNeutralMode(NeutralMode.Brake);
        lmMotor.setNeutralMode(NeutralMode.Brake);
        lbMotor.setNeutralMode(NeutralMode.Brake);
        rtMotor.setNeutralMode(NeutralMode.Brake);
        rmMotor.setNeutralMode(NeutralMode.Brake);
        rbMotor.setNeutralMode(NeutralMode.Brake);

        ltMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        lmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        lbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rtMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rmMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        rbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);

        lime = new Limelight();
        
        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()), 
            ltMotor.getSensorCollection().getIntegratedSensorPosition(), 
            rtMotor.getSensorCollection().getIntegratedSensorPosition(),
            new Pose2d());
        poseEstimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(kTrackwidthMeters), 
            new Rotation2d(getHeading()), 
            ltMotor.getSelectedSensorPosition(), 
            rtMotor.getSelectedSensorPosition(), 
            getPose()
        );
        resetEncoders();
        zeroHeading();
        isBreak = true;
        lowerCANBusUtilization();
        // prettyPrintStatusFrames();
    }

    public void lowerCANBusUtilization() {
        for (WPI_TalonFX talon: new WPI_TalonFX[]{ltMotor, lmMotor, lbMotor, rtMotor, rmMotor, rbMotor}) {
            // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
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

    public void prettyPrintStatusFrames() {
        WPI_TalonFX[] motors = {ltMotor, lmMotor, lbMotor, rtMotor, rmMotor, rbMotor};
        for (int i = 0; i < motors.length; i++) {
            // System.out.println(motorNames[i]);
            // System.out.println("\t Status_1_General: " + motors[i].getStatusFramePeriod(StatusFrame.Status_1_General) + " ms");
            // System.out.println("\t Status_2_Feedback0: " + motors[i].getStatusFramePeriod(StatusFrame.Status_2_Feedback0) + " ms");
            // System.out.println("\t Status_4_AinTempVbat: " + motors[i].getStatusFramePeriod(StatusFrame.Status_4_AinTempVbat) + " ms");
            // System.out.println("\t Status_6_Misc: " + motors[i].getStatusFramePeriod(StatusFrame.Status_6_Misc) + " ms");
            // System.out.println("\t Status_7_CommStatus: " + motors[i].getStatusFramePeriod(StatusFrame.Status_7_CommStatus) + " ms");
            // System.out.println("\t Status_10_MotionMagic: " + motors[i].getStatusFramePeriod(StatusFrame.Status_10_MotionMagic) + " ms");
            // System.out.println("\t Status_10_Targets: " + motors[i].getStatusFramePeriod(StatusFrame.Status_10_Targets) + " ms");
            // System.out.println("\t Status_12_Feedback1: " + motors[i].getStatusFramePeriod(StatusFrame.Status_12_Feedback1) + " ms");
            // System.out.println("\t Status_13_Base_PIDF0: " + motors[i].getStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0) + " ms");
            // System.out.println("\t Status_14_Turn_PIDF1: " + motors[i].getStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1) + " ms");
            // System.out.println("\t Status_15_FirmwareApiStatus: " + motors[i].getStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus) + " ms");
            // System.out.println("\t Status_17_Targets1: " + motors[i].getStatusFramePeriod(StatusFrame.Status_17_Targets1) + " ms");
        }
    }

    /**
     * Get the left motor power
     * @return The left motor power
     */
    public double getLeftPower() {
        return leftPower;
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

    /**
     * Get the right motor power
     * @return The right motor power
     */
    public double getRightPower() {
        return rightPower;
    }

    /**
     * Set the right power
     * @param pwr Value to set the power to
     */
     public void setRightPower(double pwr) {
         if (pwr > 1.0) {
             rightPower = 1.0;
             return;
         } else if (pwr < -1.0) {
             rightPower = -1.0;
             return;
         }

         rightPower = pwr;
     }

     /**
      * Set the left power
      * @param pwr Value to set the power to
      */
     public void setLeftPower(double pwr) {
        if (pwr > 1.0) {
            leftPower = 1.0;
            return;
        } else if (pwr < -1.0) {
            leftPower = -1.0;
            return;
        }

        leftPower = pwr;
     }

     /**
      * Stop the right motor
      */
     public void stopRightMotor() {
         rightPower = 0.0;
         rtMotor.stopMotor();
         rmMotor.stopMotor();
         rbMotor.stopMotor();
     }

    /**
     * Stop the left motor
     */
    public void stopLeftMotor() {
        leftPower = 0.0;
        ltMotor.stopMotor();
        lmMotor.stopMotor();
        lbMotor.stopMotor();
    }

    /**
     * Drive with default values from the joysticks
     */
    public void drive() {
        drive(rightPower, leftPower, false);
    }
    
    /**
     * Drive with custom values
     * @param rightP Right motor power
     * @param leftP Left motor power
     */
    public void drive(double rightP, double leftP, boolean shouldSquare) {
        if (invertRight) {
            rightP *= INVERT_MOTOR;
        }
        
        if (invertLeft) {
            leftP *= INVERT_MOTOR;
        }
        
        if (shouldSquare) {
            rightP *= rightP * Math.signum(rightP);
            leftP *= leftP * Math.signum(leftP);
        }
        rtMotor.set(rightP);
        rmMotor.set(rightP);
        rbMotor.set(rightP);

        ltMotor.set(leftP);
        lmMotor.set(leftP);
        lbMotor.set(leftP);
    }


    // public void driveSpeed(double speed) {
    //     double test = (ltMotor.getSelectedSensorVelocity() * Units.inchesToMeters(wheelCircumferenceInches))/ (Constants.DrivetrainConstants.gearRatio * 10.0* Constants.DrivetrainConstants.encoderTicksPerRev);
    //     // ltMotor.set(ControlMode.Velocity, (-speed / targetRPM * encoderEPR)); 
    // }

    public void checkmotors(){
        rtMotor.getSelectedSensorVelocity();
        rmMotor.getSelectedSensorVelocity();
        rbMotor.getSelectedSensorVelocity();
        ltMotor.getSelectedSensorVelocity();
        lmMotor.getSelectedSensorVelocity();
        lbMotor.getSelectedSensorVelocity();
    }
        

    /**
     * Stops the drive train
     */
    public void stop() {
        stopRightMotor();
        stopLeftMotor();
    }

    /**
     * Called periodically by the CommmandScheduler
     */
    @Override
    public void periodic() {
        //odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance()); // Update the odometry in the periodic block (gets location on field)
        //periodic called ever .02s
        odometry.update(Rotation2d.fromDegrees(getHeading()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot
     * @return The pose. (position on the field)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        //return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
        return new DifferentialDriveWheelSpeeds(ltMotor.getSelectedSensorVelocity(), rtMotor.getSelectedSensorVelocity());
    }

    /**
     * Resets the odometry to a default pose Pose2d(0, 0, new Rotation2d(0))
     * Resets the encoders, also automatically resets heading
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    /**
     * Resets the odometry to the specified pose
     * Resets the encoders, also automatically resets heading
     * @param pose The pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            Rotation2d.fromDegrees(getHeading()), 
            ltMotor.getSensorCollection().getIntegratedSensorPosition(), 
            rtMotor.getSensorCollection().getIntegratedSensorPosition(),
            pose);
    }

    /**
     * Drives the robot using arcade controls
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        //diffDrive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        ltMotor.setVoltage(leftVolts);
        lmMotor.setVoltage(leftVolts);
        lbMotor.setVoltage(leftVolts);
        rtMotor.setVoltage(-rightVolts);
        rmMotor.setVoltage(-rightVolts);
        rbMotor.setVoltage(-rightVolts);        
        odometry.update(Rotation2d.fromDegrees(gyro.getYaw()), ltMotor.getSensorCollection().getIntegratedSensorPosition(), rtMotor.getSensorCollection().getIntegratedSensorPosition());
        
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

    /**
     * Gets the average distance of the two encoders
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return ltMotor.getSensorCollection().getIntegratedSensorPosition()/2048 *( .5 * Math.PI);
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    /* public void setMaxOutput(double maxOutput) {
        diffDrive.setMaxOutput(maxOutput);
    } */

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
}