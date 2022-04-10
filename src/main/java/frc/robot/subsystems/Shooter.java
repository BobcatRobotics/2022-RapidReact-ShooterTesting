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
import frc.robot.commands.intakeControls;
import frc.robot.lib.RioLogger;

public class Shooter extends SubsystemBase {
    // TalonFXs for shooter motors
    private final WPI_TalonFX shooterFalconLeft;
    private final WPI_TalonFX shooterFalconRight;
    private final WPI_TalonFX hoodFalcon;
    private final WPI_TalonFX feedMotor;
    private final TimeOfFlight tofSensor;

    public double upperHubShootingSpeed = ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED;
    public double lowerHubShootingSpeed = ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED;
    public double hoodHigh = ShooterConstants.DEFAULT_UPPER_HUB_SHOOTING_SPEED;
    public double hoodLow = ShooterConstants.DEFAULT_LOWER_HUB_SHOOTING_SPEED;
    private boolean isRunning = false;
    private double shootingModeKey = 0;

    private double targetRPM = 600.0;
    private double encoderEPR = 2048.0;
    private double mainRPMThreshold = 50.0;
    private double hoodRPMThreshold = 50.0;

    private int ballCounter = 0;
    // private boolean isShooterSolenoidExtended;

    private int[][] originalStatusFrames = new int[3][12];

    public Shooter() {
        // Instantiate new talons
        shooterFalconLeft = new WPI_TalonFX(Constants.ShooterConstants.shooterFalcon1Port);
        shooterFalconRight = new WPI_TalonFX(Constants.ShooterConstants.shooterFalcon2Port);
        feedMotor = new WPI_TalonFX(Constants.ShooterConstants.feedMotorPort);
        hoodFalcon = new WPI_TalonFX(Constants.ShooterConstants.hoodFalconPort);
        
        // Time of flight config
        tofSensor = new TimeOfFlight(Constants.ShooterConstants.tofPort);
        tofSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 25.0);

        // Set factory default for each motor
        shooterFalconLeft.configFactoryDefault();
        shooterFalconRight.configFactoryDefault();
        feedMotor.configFactoryDefault();
        hoodFalcon.configFactoryDefault();

        // In neutral, set motors to coast
        shooterFalconLeft.setNeutralMode(NeutralMode.Coast);
        shooterFalconRight.setNeutralMode(NeutralMode.Coast);
        feedMotor.setNeutralMode(NeutralMode.Brake);
        hoodFalcon.setNeutralMode(NeutralMode.Coast);

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

        hoodFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        hoodFalcon.setSelectedSensorPosition(0, 0, 0);
        hoodFalcon.setSensorPhase(false);

        // setup pid values
        shooterFalconLeft.config_kF(0, 0.05, 0);
        shooterFalconLeft.config_kP(0, 0.11616, 0);
        shooterFalconLeft.config_kI(0, 0.00002, 0);
        shooterFalconLeft.config_kD(0, 0.0, 0);
        shooterFalconLeft.config_IntegralZone(0, 3, 0);
        shooterFalconRight.config_kF(0, 0.05, 0);
        shooterFalconRight.config_kP(0, 0.11616, 0);
        shooterFalconRight.config_kI(0, 0.00002, 0);
        shooterFalconRight.config_kD(0, 0.0, 0);
        shooterFalconRight.config_IntegralZone(0, 3, 0);
        hoodFalcon.config_kF(0, 0.05, 0);
        hoodFalcon.config_kP(0, 0.16421, 0);
        hoodFalcon.config_kI(0, 0.00002, 0);
        hoodFalcon.config_kD(0, 0.0, 0);
        hoodFalcon.config_IntegralZone(0, 3, 0);

        // shooterAngleSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ShooterConstants.shooterAngleSolenoidPort);
        // shooterAngleSolenoid.set(false);
        // isShooterSolenoidExtended = shooterAngleSolenoid.get();
    }

    private double tof_thresh = 30;

    public double getTofThresh() {
        return tof_thresh;
    }

    public void setTofThresh(double thresh) {
        tof_thresh = thresh;
    }

    public boolean tofTriggered() {
        return tofSensor.getRange() <= getTofThresh();
    }

    public double getTofRange() {
        return tofSensor.getRange();
    }

    public void setMainRPMThreshold(double thresh) {
        mainRPMThreshold = thresh;
    }
    public double getMainRPMThreshold() {
        return mainRPMThreshold;
    }
    public void setHoodRPMThreshold(double thresh) {
        hoodRPMThreshold = thresh;
    }
    public double getHoodRPMThreshold() {
        return hoodRPMThreshold;
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
        double[] speeds = convertShootingModeKeyToShootingRPM(shootingModeKey);
        double mainRPM = speeds[0], hoodRPM = speeds[1];
        shooterFalconLeft.set(ControlMode.Velocity, (-mainRPM / targetRPM * encoderEPR));
        shooterFalconRight.set(ControlMode.Velocity, (-mainRPM / targetRPM * encoderEPR));
        hoodFalcon.set(ControlMode.Velocity, (hoodRPM / targetRPM * encoderEPR));
    }

    public void stop() {
        stopShooter();
        stopFeeding();
    }

    // Stop shooter motors
    public void stopShooter() {
        shooterFalconLeft.stopMotor();
        shooterFalconRight.stopMotor();
        hoodFalcon.stopMotor();
    }

    // Get right RPM
    public double getRightRPM() {
        return shooterFalconRight.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

    // Get left RPM
    public double getLeftRPM() {
        return shooterFalconLeft.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

// Get left RPM
    public double getHoodRPM() {
        return hoodFalcon.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

    // Set high speed
    public void setHighSpeed(double speed) {
        ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.UPPER_HUB_KEY)[0] = speed;
    }
    // Set high hood speed
    public void setHighHoodSpeed(double speed) {
        ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.UPPER_HUB_KEY)[1] = speed;
    }
    
    // Set low speed
    public void setLowSpeed(double speed) {
        ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.LOWER_HUB_KEY)[0] = speed;
    }

    // Set low hood speed
    public void setLowHoodSpeed(double speed) {
        ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.LOWER_HUB_KEY)[1] = speed;
    }
    

    public double[] convertShootingModeKeyToShootingRPM(double key) {
        return ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(key);
    }

    public double[] getDefaultUpperHubShootingSpeeds() {
        return ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.UPPER_HUB_KEY);
    }
    
    public double[] getDefaultLowerHubShootingSpeeds() {
        return ShooterConstants.LIMELIGHT_SHOOTING_LOOKUP_MAP.get(ShooterConstants.LOWER_HUB_KEY);
    }

    // Check if motor is at speed
    public boolean atSpeed() {
        double[] speeds = convertShootingModeKeyToShootingRPM(shootingModeKey);
        double mainRPM = speeds[0], hoodRPM = speeds[1];
        return (
            ((Math.abs(getRightRPM()) >= (mainRPM - mainRPMThreshold)) || (getLeftRPM() >= (mainRPM - mainRPMThreshold)))
            && (Math.abs(getHoodRPM()) >= (hoodRPM - hoodRPMThreshold))
        );
        // return true;
    }

    // Set manual / limelight shooting operation
    public void setShootingModeKey(double key) {
        shootingModeKey = key;
    }

    // Get manual / limelight shooting operation
    public double getShootingModeKey() {
        return shootingModeKey;
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
    // public boolean getBallReadyToFeed() {
    //     return false;
    // }

    // public boolean getBallLeaving() {
    //     return false;
    // }

    // public void toggleShooter() {
    // }

    // public void shooterUp() {
    // }

    // public void shooterDown() {
    // }

    // public void setShooterSolenoidExtended(boolean status) {
    //     // shooterAngleSolenoid.set(status);
    //     isShooterSolenoidExtended = status;
    // }

    // public boolean isShooterSolenoidExtended() {
    //     return isShooterSolenoidExtended;
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
        }
    }

}
