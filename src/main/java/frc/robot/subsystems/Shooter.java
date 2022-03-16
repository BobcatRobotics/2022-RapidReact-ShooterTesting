package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // Compressor
    // private Compressor compressorModel;

    // TalonFXs for shooter motors
    private final WPI_TalonFX shooterFalconLeft;
    private final WPI_TalonFX shooterFalconRight;
    private final WPI_TalonFX feedMotor;

    public double upperHubShootingSpeed = ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED;
    public double lowerHubShootingSpeed = ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED;
    private final double targetSpeed = 3000.0;
    private boolean isRunning = false;

    private double targetRPM = 600.0;
    private double encoderEPR = 2048.0;
    private final double rpmThreshold = 500.0;

    // Feeder Motor & Sensors & Other ****
    // private final TimeOfFlight ballPresentSensor;
    // private final TimeOfFlight ballLeavingSensor;
    private int ballCounter = 0;
    private final Solenoid shooterAngleSolenoid;
    private boolean isShooterSolenoidExtended;

    private int[][] originalStatusFrames = new int[3][12];

    private boolean highMode = true;
    public void setHighMode(boolean status) {highMode = status;}

    public Shooter() {
        // Init new compressor object from port in Constants
        // compressorModel = new Compressor(ShooterConstants.compressorModelPort,
        // PneumaticsModuleType.REVPH);

        // Instantiate new talons
        shooterFalconLeft = new WPI_TalonFX(Constants.ShooterConstants.shooterFalcon1Port);
        shooterFalconRight = new WPI_TalonFX(Constants.ShooterConstants.shooterFalcon2Port);
        feedMotor = new WPI_TalonFX(Constants.ShooterConstants.feedMotorPort);

        // Set factory default for each motor
        shooterFalconLeft.configFactoryDefault();
        shooterFalconRight.configFactoryDefault();
        feedMotor.configFactoryDefault();

        // In neutral, set motors to coast
        shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
        shooterFalconRight.setNeutralMode(NeutralMode.Coast);
        feedMotor.setNeutralMode(NeutralMode.Brake);

        // Set both falcon motor speeds equal to each other
        // shooterFalconRight.follow(shooterFalconLeft);
        // shooterFalconLeft.follow(shooterFalconRight);

        // Invert one motor because motors will be facing
        // each other, meaning that one will be rotating in
        // the opposite direction of the other.
        shooterFalconLeft.setInverted(true);
        shooterFalconRight.setInverted(false);

        // setup sensors
        shooterFalconLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        shooterFalconLeft.setSelectedSensorPosition(0, 0, 0);
        shooterFalconLeft.setSensorPhase(false);
        shooterFalconRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        shooterFalconRight.setSelectedSensorPosition(0, 0, 0);
        shooterFalconRight.setSensorPhase(false);

        // setup pid values
        shooterFalconLeft.config_kF(0, 0.047, 0);
        shooterFalconLeft.config_kP(0, 0.0015, 0);
        shooterFalconLeft.config_kI(0, 0.00002, 0);
        shooterFalconLeft.config_kD(0, 0.0, 0);
        shooterFalconLeft.config_IntegralZone(0, 3, 0);
        shooterFalconRight.config_kF(0, 0.047, 0);
        shooterFalconRight.config_kP(0, 0.0015, 0);
        shooterFalconRight.config_kI(0, 0.00002, 0);
        shooterFalconRight.config_kD(0, 0.0, 0);
        shooterFalconRight.config_IntegralZone(0, 3, 0);

        // TODO: Need to do any sensor/voltage/solenoid stuff?

        // Feeder stuff
        // ballPresentSensor = new TimeOfFlight(FeederConstants.feederBallPresentId);
        // ballLeavingSensor = new TimeOfFlight(FeederConstants.feederBallLeavingId);
        shooterAngleSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ShooterConstants.shooterAngleSolenoidPort);
        shooterAngleSolenoid.set(false);
        isShooterSolenoidExtended = shooterAngleSolenoid.get();

        getToSpeed();
        // lowerCANBusUtilization();
        // prettyPrintStatusFrames();
    }

    // public void lowerCANBusUtilization() {
    //     for (WPI_TalonFX talon: new WPI_TalonFX[]{shooterFalconLeft, shooterFalconRight, feedMotor}) {
    //         // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
    //         talon.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    //         talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    //         talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_6_Misc, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 384);
    //         talon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 384);
    //     }
    // }

    public void lowerCANBusUtilization() {
        // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
        for (WPI_TalonFX talon: new WPI_TalonFX[]{shooterFalconLeft, shooterFalconRight, feedMotor}) {
            for (int i = 0; i < originalStatusFrames.length; i++) {
                talon.setStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i]);
            }
        }
        prettyPrintStatusFrames();
    }

    public void configDefault() {
        WPI_TalonFX[] talons = new WPI_TalonFX[]{shooterFalconLeft, shooterFalconRight, feedMotor};
        for (int t = 0; t < talons.length; t++) {
            for (int i = 0; i < originalStatusFrames.length; i++) {
                originalStatusFrames[t][i] = talons[t].getStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i]);
            }
        }
    }

    public void defaultStatusFrames() {
        WPI_TalonFX[] talons = new WPI_TalonFX[]{shooterFalconLeft, shooterFalconRight, feedMotor};
        for (int t = 0; t < talons.length; t++) {
            for (int i = 0; i < originalStatusFrames.length; i++) {
                talons[t].setStatusFramePeriod(Constants.frameTypes[i], originalStatusFrames[t][i]);
            }
        }
    }

    public void prettyPrintStatusFrames() {
        WPI_TalonFX[] motors = {shooterFalconLeft, shooterFalconRight, feedMotor};
        String[] motorNames = {"shooterFalconLeft", "shooterFalconRight", "feedMotor"};
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

    public double getLeftCurrent() {
        return shooterFalconLeft.getStatorCurrent();
    }

    public double getRightCurrent() {
        return shooterFalconRight.getStatorCurrent();
    }

    public double getLeftVoltage() {
        return shooterFalconLeft.getMotorOutputVoltage();
    }
    
    public double getRightVoltage() {
        return shooterFalconRight.getMotorOutputVoltage();
    }

    // Start shooter motors
    public void getToSpeed() {
        double s = highMode ? upperHubShootingSpeed : lowerHubShootingSpeed;
        // double s = upperHubShootingSpeed;
        // shooterFalconLeft.set(-0.7);
        // shooterFalconLeft.set(ControlMode.Velocity, 4800);
        // // System.out.println("getting to speed: " + (-(speed/targetRPM*encoderEPR)));
        shooterFalconLeft.set(ControlMode.Velocity, (-s / targetRPM * encoderEPR));
        shooterFalconRight.set(ControlMode.Velocity, (-s / targetRPM * encoderEPR));
    }

    public void stop() {
        stopShooter();
        stopFeeding();
    }

    // Stop shooter motors
    public void stopShooter() {
        shooterFalconLeft.stopMotor();
        shooterFalconRight.stopMotor();
        // shooterFalconLeft.set(0);
        // shooterFalconRight.set(0);
    }

    // Get right RPM
    public double getRightRPM() {
        return shooterFalconRight.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

    // Get left RPM
    public double getLeftRPM() {
        return shooterFalconLeft.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

    // Set high speed
    public void setHighSpeed(double speed) {
        this.upperHubShootingSpeed = speed;
    }

    // Set low speed
    public void setLowSpeed(double speed) {
        this.lowerHubShootingSpeed = speed;
    }

    // Get speed
    public double getSpeed() {
        return upperHubShootingSpeed;
    }

    // Check if motor is at speed
    public boolean atSpeed() {
        double s = highMode ? upperHubShootingSpeed : lowerHubShootingSpeed;
        // double s = upperHubShootingSpeed;
        return ((Math.abs(getRightRPM()) >= (s - rpmThreshold)) || (getLeftRPM() >= (s - rpmThreshold)));
    }

    // Check if shooter is running
    public boolean isRunning() {
        return isRunning;
    }

    // Set shooter running boolean
    public void setRunning(boolean state) {
        isRunning = state;
    }

    public void feed() {
        feed(true);
    }

    public void feed(boolean fullSpeed) {
        if (fullSpeed) {
            feedMotor.set(-FeederConstants.speedLimiter);
        } else {
            feedMotor.set(-FeederConstants.speedLimiterSlow);
        }
    }

    public void reverseFeed() {
        feedMotor.set(FeederConstants.speedLimiter);
    }

    public void stopFeeding() {
        feedMotor.stopMotor();
    }

    /**
     * Increments the ball count by 1.
     */
    public void addBall() {
        ballCounter++;
    }

    /**
     * Decrements the ball count by 1.
     */
    public void shotBall() {
        ballCounter--;
    }

    public int getCounter() {
        return ballCounter;
    }

    /**
     * Returns whether or not the TOF sensor currently sees a ball
     */
    public boolean getBallReadyToFeed() {
        return false;
        // if(ballPresentSensor == null)
        // return false;
        // double range = ballPresentSensor.getRange();
        // boolean ballPresent = false;
        // if (range <= FeederConstants.feederBallPresentThreshold) {
        // ballPresent = true;
        // }
        // return ballPresent;
    }

    public boolean getBallLeaving() {
        return false;
        // if (ballLeavingSensor == null) {
        // return false;
        // }
        // double range = ballLeavingSensor.getRange();
        // boolean ballLeaving = false;
        // if (range <= FeederConstants.feederBallLeavingThreshold) {
        // ballLeaving = true;
        // }
        // return ballLeaving;
    }

    public void toggleShooter() {
        // boolean solenoidState = pneumaticCylinder.get();
        // solenoidState ? shooterDown() : shooterUp();
    }

    public void shooterUp() {
        // pneumaticCylinder.set(false);
    }

    public void shooterDown() {
        // pneumaticCylinder.set(true);
    }

    public void setShooterSolenoidExtended(boolean status) {
        shooterAngleSolenoid.set(status);
        isShooterSolenoidExtended = status;
    }

    public boolean isShooterSolenoidExtended() {
        return isShooterSolenoidExtended;
    }

    // [PR] May not need stuff below because it is in intake - although we might
    // have to call that particular toggleCompressor() method in Robot.java
    // /**
    // * Toggles compressor and returns new state
    // */
    // public boolean toggleCompressor() {
    // if (compressorModel.enabled()) compressorModel.disable();
    // else compressorModel.enableDigital();
    // return compressorModel.enabled();
    // }

    // /**
    // * Returns state of compressor
    // */
    // public boolean compressorIsEnabled() {
    // return compressorModel.enabled();
    // }
}
