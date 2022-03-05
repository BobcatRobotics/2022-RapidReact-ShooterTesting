package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.RioLogger;

public class Intake extends SubsystemBase {
    // Compressor
    private Compressor compressorModel;
    private PneumaticHub phub = new PneumaticHub(1);

    // Horizontal BAG wheels
    private WPI_VictorSPX intakeLeftWheel;

    // Falcon intake
    private WPI_TalonFX intakeBar;

    // Solenoids
    private Solenoid intakeSolenoid;
    private boolean isDeployed;

    // Target speeds - we can tune these values as needed
    private double inFullSpeed = 1.0;
    private double inHalfSpeed = 0.3;
    private double outSpeed = -0.5;

    private int[][] originalStatusFrames = new int[2][12];

    /**
     * Constructor for the Intake subsystem
     */
    public Intake() {
        // Init new compressor object from port in Constants
        compressorModel = new Compressor(compressorModelPort, PneumaticsModuleType.REVPH);

        // Init intake motors
        intakeLeftWheel = new WPI_VictorSPX(intakeLeftWheelPort);
        // intakeRightWheel = new WPI_TalonSRX(intakeRightWheelPort);
        // intakeRightWheel.setInverted(true); // Based on CAD drawing, the wheels should spin in opposite directions
        intakeBar = new WPI_TalonFX(intakeBarPort);

        // Init solenoids
        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, intakeSolenoidPort);
        intakeSolenoid.set(false);
        isDeployed = intakeSolenoid.get();

        // When in neutral mode for intake motors, allow for coasting
        intakeLeftWheel.setNeutralMode(NeutralMode.Coast);
        // intakeRightWheel.setNeutralMode(NeutralMode.Coast);
        intakeBar.setNeutralMode(NeutralMode.Coast);

        // prettyPrintStatusFrames();
        // configDefault();
        // defaultStatusFrames();
        lowerCANBusUtilization();
    }

    public void feedOut() {
        runIntakeBarOut(true);
        runIntakeWheelsOut(true);
    }
    public void feedIn() {
        runIntakeBarIn(true);
        runIntakeWheelsIn(true);   
    }

    public void lowerCANBusUtilization() {
        // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
        for (int i = 0; i < originalStatusFrames.length; i++) {
            intakeLeftWheel.setStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i], Constants.kTimeoutMs);
            intakeBar.setStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i], Constants.kTimeoutMs);
        }
        prettyPrintStatusFrames();
    }

    public void configDefault() {
        for (int i = 0; i < originalStatusFrames.length; i++) {
            originalStatusFrames[0][i] = intakeLeftWheel.getStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i]);
            originalStatusFrames[1][i] = intakeBar.getStatusFramePeriod(Constants.frameTypes[i], Constants.desiredStatusFrames[i]);
        }
    }

    public void defaultStatusFrames() {
        for (int i = 0; i < originalStatusFrames.length; i++) {
            originalStatusFrames[0][i] = intakeLeftWheel.getStatusFramePeriod(Constants.frameTypes[i], originalStatusFrames[0][i]);
            originalStatusFrames[1][i] = intakeBar.getStatusFramePeriod(Constants.frameTypes[i], originalStatusFrames[1][i]);
        }
    }
    // public void lowerCANBusUtilization() {
    //     // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_6_Misc, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_10_Targets, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 384);
    //     intakeLeftWheel.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 384);
    // }

    public void prettyPrintStatusFrames() {
        System.out.println("Intake wheel:");
        System.out.println("\t Status_1_General: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_1_General) + " ms");
        System.out.println("\t Status_2_Feedback0: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_2_Feedback0) + " ms");
        System.out.println("\t Status_4_AinTempVbat: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_4_AinTempVbat) + " ms");
        System.out.println("\t Status_6_Misc: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_6_Misc) + " ms");
        System.out.println("\t Status_7_CommStatus: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_7_CommStatus) + " ms");
        System.out.println("\t Status_10_MotionMagic: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_10_MotionMagic) + " ms");
        System.out.println("\t Status_10_Targets: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_10_Targets) + " ms");
        System.out.println("\t Status_12_Feedback1: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_12_Feedback1) + " ms");
        System.out.println("\t Status_13_Base_PIDF0: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0) + " ms");
        System.out.println("\t Status_14_Turn_PIDF1: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1) + " ms");
        System.out.println("\t Status_15_FirmwareApiStatus: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus) + " ms");
        System.out.println("\t Status_17_Targets1: " + intakeLeftWheel.getStatusFramePeriod(StatusFrame.Status_17_Targets1) + " ms");
        System.out.println("Intake bar:");
        System.out.println("\t Status_1_General: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_1_General) + " ms");
        System.out.println("\t Status_2_Feedback0: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_2_Feedback0) + " ms");
        System.out.println("\t Status_4_AinTempVbat: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_4_AinTempVbat) + " ms");
        System.out.println("\t Status_6_Misc: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_6_Misc) + " ms");
        System.out.println("\t Status_7_CommStatus: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_7_CommStatus) + " ms");
        System.out.println("\t Status_10_MotionMagic: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_10_MotionMagic) + " ms");
        System.out.println("\t Status_10_Targets: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_10_Targets) + " ms");
        System.out.println("\t Status_12_Feedback1: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_12_Feedback1) + " ms");
        System.out.println("\t Status_13_Base_PIDF0: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0) + " ms");
        System.out.println("\t Status_14_Turn_PIDF1: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1) + " ms");
        System.out.println("\t Status_15_FirmwareApiStatus: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus) + " ms");
        System.out.println("\t Status_17_Targets1: " + intakeBar.getStatusFramePeriod(StatusFrame.Status_17_Targets1) + " ms");
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

        } else if (pressure <= 90) {
            // compressorModel.enableDigital();
            compressorModel.enableAnalog(90, 115);
        }
    }

    /**
     * Runs the intake wheels inward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeWheelsIn(boolean fullSpeed) {
        intakeLeftWheel.set(fullSpeed ? inFullSpeed : inHalfSpeed);
        // intakeRightWheel.set(fullSpeed ? inFullSpeed : inHalfSpeed);
    }

    /**
     * Runs the intake wheels outward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeWheelsOut(boolean fullSpeed) {
        intakeLeftWheel.set(outSpeed);
        // intakeRightWheel.set(outSpeed);
    }

    /**
     * Runs the intake bar motor inward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeBarIn(boolean fullSpeed) {
        intakeBar.set(fullSpeed ? inFullSpeed : inHalfSpeed);
    }

    /**
     * Runs the intake bar motor outward
     * @param fullSpeed True if we want to run at full speed, false if we want to run at half speed
     */
    public void runIntakeBarOut(boolean fullSpeed) {
        intakeBar.set(outSpeed);
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
        // // intakeRightWheel.set(0);
    }

    /**
     * Stops the intake bar motor
     */
    public void stopIntakeBar() {
        intakeBar.set(0);
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
        if (compressorModel.enabled()) compressorModel.disable();
        else compressorModel.enableDigital();
        return compressorModel.enabled();
    }

    /**
     * Returns state of compressor
     */
    public boolean compressorIsEnabled() {
        return compressorModel.enabled();
    }
    
}
