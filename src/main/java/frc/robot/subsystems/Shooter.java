package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    // TalonFXs for shooter motors
    private final WPI_TalonFX shooterFalconLeft;
    private final WPI_TalonFX shooterFalconRight;
    private final WPI_TalonFX feedMotor;

    private double speed = 4800.0;
    private final double targetSpeed = 4800.0;
    private boolean isRunning = false;

    private double targetRPM = 600.0;
    private double encoderEPR = 2048.0;
    private final double rpmThreshold = 800.0;

    // Feeder Motor & Sensors & Other ****
    // private final TimeOfFlight ballPresentSensor;
    // private final TimeOfFlight ballLeavingSensor;
    private int ballCounter = 0;
    // private final Solenoid pneumaticCylinder;

    public Shooter() {
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
        shooterFalconLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        shooterFalconLeft.setSelectedSensorPosition(0,0,0);
        shooterFalconLeft.setSensorPhase(false);
        shooterFalconRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        shooterFalconRight.setSelectedSensorPosition(0,0,0);
        shooterFalconRight.setSensorPhase(false);

        //setup pid values
        shooterFalconLeft.config_kF(0,0.047,0);
        shooterFalconLeft.config_kP(0,0.0015,0);
        shooterFalconLeft.config_kI(0,0.00002,0);
        shooterFalconLeft.config_kD(0,0.0,0);
        shooterFalconLeft.config_IntegralZone(0,3,0);
        shooterFalconRight.config_kF(0,0.047,0);
        shooterFalconRight.config_kP(0,0.0015,0);
        shooterFalconRight.config_kI(0,0.00002,0);
        shooterFalconRight.config_kD(0,0.0,0);
        shooterFalconRight.config_IntegralZone(0,3,0);
        
        // TODO: Need to do any sensor/voltage/solenoid stuff?

        // Feeder stuff
        // ballPresentSensor = new TimeOfFlight(FeederConstants.feederBallPresentId);
        // ballLeavingSensor = new TimeOfFlight(FeederConstants.feederBallLeavingId);
        // pneumaticCylinder = new Solenoid(PneumaticsModuleType.REVPH, ShooterConstants.shooterAngleSolenoid);
    }

    // Start shooter motors
    public void getToSpeed() {
        // shooterFalconLeft.set(-0.7);
        // shooterFalconLeft.set(ControlMode.Velocity, 4800);
        System.out.println("getting to speed: " + (-(speed/targetRPM*encoderEPR)));
        shooterFalconLeft.set(ControlMode.Velocity, (-speed/targetRPM*encoderEPR));
        shooterFalconRight.set(ControlMode.Velocity, (-speed/targetRPM*encoderEPR));
    }

    public void stop() {
        stopShooter();
        stopFeeding();
    }

    // Stop shooter motors
    public void stopShooter() {
        shooterFalconLeft.stopMotor();
        shooterFalconRight.stopMotor();
    }

    // Get right RPM
    public double getRightRPM() {
        return shooterFalconRight.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }
    
    // Get left RPM
    public double getLeftRPM() {
        return shooterFalconLeft.getSelectedSensorVelocity() * targetRPM / encoderEPR;
    }

    // Set speed
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    // Get speed
    public double getSpeed() {
        return speed;
    }

    // Check if motor is at speed
    public boolean atSpeed() {
        return ((getRightRPM() >= (speed - rpmThreshold)) || (getLeftRPM() >= (speed - rpmThreshold)));
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
            feedMotor.set(FeederConstants.speedLimiter);
        } else {
            feedMotor.set(FeederConstants.speedLimiterSlow);
        }
    }

    public void reverseFeed() {
        feedMotor.set(-FeederConstants.speedLimiter);
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
        //     return false;
        // double range = ballPresentSensor.getRange();
        // boolean ballPresent = false;
        // if (range <= FeederConstants.feederBallPresentThreshold) {
        //     ballPresent = true;
        // }
        // return ballPresent;
    }

    public boolean getBallLeaving() {
        return false;
        // if (ballLeavingSensor == null) {
        //     return false;
        // }
        // double range = ballLeavingSensor.getRange();
        // boolean ballLeaving = false;
        // if (range <= FeederConstants.feederBallLeavingThreshold) {
        //     ballLeaving = true;
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

    public void setShooter(boolean position) {
        // pneumaticCylinder.set(position);
    }
}
