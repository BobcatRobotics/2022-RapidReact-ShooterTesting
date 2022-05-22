package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.PropertyNamingStrategies.LowerCamelCaseStrategy;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // private Compressor compressorModel;

    private WPI_TalonFX winchMotor;
    private DigitalInput leftWinchSwitch;
    private DigitalInput rightWinchSwitch;

    // private Solenoid climberSolenoid;

    private boolean deployed = false;
    private boolean isClimberMode = false;
    private boolean didResetSoftLimit = false;

    private double zeroReference = 0;
    private double climbMotorSpeedLimiter = 1.0;

    private double fullClimbingSpeed = 0.5;

    private int[] originalStatusFrames = new int[12];

    public Climber() {
        // compressorModel = new Compressor(compressorModelPort,
        // PneumaticsModuleType.REVPH);

        winchMotor = new WPI_TalonFX(winchMotorPort);

        winchMotor.configFactoryDefault();
        winchMotor.setNeutralMode(NeutralMode.Brake);
        winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        leftWinchSwitch = new DigitalInput(leftWinchSwitchPort);
        rightWinchSwitch = new DigitalInput(rightWinchSwitchPort);

        // climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, climberSolenoidPort);
        // if (climberSolenoid.get()) {
            // System.out.println("Climber pistons have been extended to start with.");
        // }

        // configDefault();
        // defaultStatusFrames();
        isClimberMode = false;

        SmartDashboard.putBoolean("CLIMB: Enable soft limits", true);

        if (SmartDashboard.getBoolean("CLIMB: Enable soft limits", true)) {
            resetSoftLimitIfNeeded();
        }

        lowerCANBusUtilization();
    }

    public void lowerCANBusUtilization() {
        // [PR] BUG FIX ATTEMPT FOR >70% CAN BUS UTILIZATION - REDUCE COMMUNICATION FREQUENCIES FOR UNUSED MOTOR OUTPUT
        winchMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255, Constants.kTimeoutMs);
        winchMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255, Constants.kTimeoutMs);
    }

    public void resetSoftLimitIfNeeded() {
        if (switchTripped() || !didResetSoftLimit) {
            zeroReference = winchMotor.getSelectedSensorPosition();
            // Figure out native units in Phoenix Tuner
            winchMotor.configReverseSoftLimitThreshold(-255000+zeroReference, 0);
            // winchMotor.configReverseSoftLimitThreshold(-100000, 0);
            winchMotor.configReverseSoftLimitEnable(true, 0);
            didResetSoftLimit = true;
        }
    }

    public void toggleSwitchToClimberMode() {
        isClimberMode = !isClimberMode;
    }

    public boolean isClimberMode() {
        return isClimberMode;
    }

    // public void deploy() {
    //     climberSolenoid.set(true);
    //     deployed = true;
    // }

    public boolean bothSwitchesTripped() {
        return !(leftWinchSwitch.get() || rightWinchSwitch.get());
    }

    public boolean switchTripped() {
        return !(leftWinchSwitch.get() && rightWinchSwitch.get());
    }

    public void climb(boolean isUnwinding) {
        // If either limit switch has been hit and the climber is being commanded
        // to go down, stop the winch motor 
        if (switchTripped() && !isUnwinding) {
            winchMotor.stopMotor();
        }
        // Otherwise, if neither limit switch is being pressed or
        // the climber is being commanded to go up, go full speed
        else {
            winchMotor.set(isUnwinding ? -fullClimbingSpeed : fullClimbingSpeed);
        }
    }

    public void turnOffClimberMode() {
        isClimberMode = false;
    }

    public void climb(Double climbSpeed) {
        // If some idiot decides to give too large or too small
        // of a climber speed, limit it to between -1 and 1.
        if (climbSpeed > 1) climbSpeed = 1.0;
        if (climbSpeed < -1) climbSpeed = -1.0;

        // If the joystick is drifting for some reason at rest,
        // stop the winch motor.
        if (Math.abs(climbSpeed) < 0.01) {
            // climbSpeed = 0.0;
            stop();
            return;
        }
        // If either limit switch has been hit and the climber is being commanded
        // to go down, stop the winch motor
        if (switchTripped() && climbSpeed > 0) {
            winchMotor.stopMotor();
            if (SmartDashboard.getBoolean("CLIMB: Enable soft limits", false)) {
                resetSoftLimitIfNeeded();
            }
        }
        // Otherwise, if neither limit switch is being pressed or
        // the climber is being commanded to go up, go full speed
        else {
            winchMotor.set(climbSpeed);
        }

        // winchMotor.set(climbSpeed * climbMotorSpeedLimiter);
    }

    // public void withdraw() {
    //     climberSolenoid.set(false);
    //     deployed = false;
    // }

    public void stop() {
        winchMotor.stopMotor();
    }

    public boolean isDeployed() {
        return deployed;
    }
}