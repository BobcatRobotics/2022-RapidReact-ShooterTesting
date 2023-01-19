package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Compressor
    private Compressor compressorModel;
    private PneumaticHub phub = new PneumaticHub(1);

    // Horizontal BAG wheels
    private WPI_VictorSPX intakeLeftWheel;
    private WPI_VictorSPX intakeRightWheel;

    // Falcon intake
    private WPI_TalonFX intakeBar;

    // Solenoids
    private Solenoid intakeSolenoid;
    private boolean isDeployed;

    // Target speeds - we can tune these values as needed
    private double inFullSpeed = 1.0;
    private double inHalfSpeed = 0.3;
    private double outSpeed = -0.5;

    private boolean intakeBarRunningIn = false;
    private boolean intakeWheelsRunningIn = false;
   

    /**
     * Constructor for the Intake subsystem
     */
    public Intake() {
        // Init new compressor object from port in Constants
        compressorModel = new Compressor(compressorModelPort, PneumaticsModuleType.REVPH);

        // Init intake motors
        intakeLeftWheel = new WPI_VictorSPX(intakeLeftWheelPort);
        intakeRightWheel = new WPI_VictorSPX(intakeRightWheelPort);
        intakeLeftWheel.setInverted(true);
        intakeRightWheel.follow(intakeLeftWheel);
        intakeBar = new WPI_TalonFX(intakeBarPort);

        // Init solenoids
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, intakeSolenoidPort);
        intakeSolenoid.set(false);
        isDeployed = intakeSolenoid.get();

        // When in neutral mode for intake motors, allow for coasting
        intakeLeftWheel.setNeutralMode(NeutralMode.Coast);
        // intakeRightWheel.setNeutralMode(NeutralMode.Coast);
        intakeBar.setNeutralMode(NeutralMode.Coast);

    }

    public void feedOut() {
        runIntakeBarOut(true);
        runIntakeWheelsOut(true);
    }
    public void feedIn() {
        runIntakeBarIn(true);
        runIntakeWheelsIn(true);   
    }

    public boolean isIntakeBarRunningIn() {
        return intakeBarRunningIn;
    }

    public boolean isIntakeWheelsRunningIn() {
        return intakeWheelsRunningIn;
    }

    public boolean isIntakeRunningIn() {
        return isIntakeBarRunningIn() || isIntakeWheelsRunningIn();
    }

    public PneumaticHub pneumaticHub() {
        return phub;
    }
    
    @Override
    public void periodic() {
        double pressure = phub.getPressure(0);
        SmartDashboard.putNumber("Compressor pressure", pressure);
        if (pressure >= 115) {
            compressorModel.disable();

        } else if (pressure <= 80) {
            // compressorModel.enableDigital();
            compressorModel.enableAnalog(80, 115);
        }
    }

    /**
     * Runs the intake wheels inward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeWheelsIn(boolean fullSpeed) {
        intakeLeftWheel.set(fullSpeed ? inFullSpeed : inHalfSpeed);
        intakeWheelsRunningIn = true;
    }

    /**
     * Runs the intake wheels outward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeWheelsOut(boolean fullSpeed) {
        intakeLeftWheel.set(outSpeed);
        intakeWheelsRunningIn = false;
    }

    /**
     * Runs the intake bar motor inward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeBarIn(boolean fullSpeed) {
        intakeBar.set(fullSpeed ? inFullSpeed : inHalfSpeed);
        intakeBarRunningIn = true;
    }

    /**
     * Runs the intake bar motor outward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeBarOut(boolean fullSpeed) {
        intakeBar.set(outSpeed);
        intakeBarRunningIn = false;
    }
    
    /**
     * Returns the current speed of the intake bar motor
     */
    public double getIntakeBarSpeed() {
        return intakeBar.get();
    }

    /**
     * Returns the current speed of the left intake wheel
     * @return
     */
    public double getIntakeLeftWheelSpeed() {
        return intakeLeftWheel.get();
    }

    /**
     * Returns the current speed of the right intake wheel
     */
    public double getintakeRightWheelSpeed() {
        //return // intakeRightWheel.get();
        return 0.0;
    }

    /**
     * Stops the intake wheels
     */
    public void stopIntakeWheels() {
        intakeLeftWheel.set(0);
        intakeWheelsRunningIn = false;
    }

    /**
     * Stops the intake bar motor
     */
    public void stopIntakeBar() {
        intakeBar.set(0);
        intakeBarRunningIn = false;
    }

    /**
     * Stops all intake motors
     */
    public void stopIntake() {
        stopIntakeWheels();
        stopIntakeBar();
    }

    /**
     * Deploys or retracts intake solenoids based on a provided desired state
     * @param state True if we want to deploy, false if we want to retract
     */
    public void deploy(boolean state) {
        intakeSolenoid.set(state); // Deployed = intake down
        isDeployed = state;
    }

    /**
     * Returns whether the intake is deployed or not
     */
    public boolean isDeployed() {
        return isDeployed;
    }

    /**
     * Toggles compressor and returns new state
     */
    public boolean toggleCompressor() {
        if (compressorModel.isEnabled()) compressorModel.disable();
        else compressorModel.enableDigital();
        return compressorModel.isEnabled();
    }

    /**
     * Returns state of compressor
     */
    public boolean compressorIsEnabled() {
        return compressorModel.isEnabled();
    }   
}