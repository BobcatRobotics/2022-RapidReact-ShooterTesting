package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // private Compressor compressorModel;

    private WPI_TalonFX winchMotor;
    private DigitalInput leftWinchSwitch;
    private DigitalInput rightWinchSwitch;

    private Solenoid climberSolenoid;

    private boolean deployed = false;
    private boolean isClimberMode = false;

    private double climbMotorSpeedLimiter = 1.0;

    private double fullClimbingSpeed = 0.5;

    public Climber() {
        // compressorModel = new Compressor(compressorModelPort,
        // PneumaticsModuleType.REVPH);

        winchMotor = new WPI_TalonFX(winchMotorPort);

        winchMotor.configFactoryDefault();
        winchMotor.setNeutralMode(NeutralMode.Brake);
        winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        leftWinchSwitch = new DigitalInput(leftWinchSwitchPort);
        rightWinchSwitch = new DigitalInput(rightWinchSwitchPort);

        climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, climberSolenoidPort);
        if (climberSolenoid.get()) {
            System.out.println("Climber pistons have been extended to start with.");
        }
    }

    public void toggleSwitchToClimberMode() {
        isClimberMode = !isClimberMode;
    }

    public boolean isClimberMode() {
        return isClimberMode;
    }

    public void deploy() {
        climberSolenoid.set(true);
        deployed = true;
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
        }
        // Otherwise, if neither limit switch is being pressed or
        // the climber is being commanded to go up, go full speed
        else {
            winchMotor.set(climbSpeed);
        }

        // winchMotor.set(climbSpeed * climbMotorSpeedLimiter);
    }

    public void withdraw() {
        climberSolenoid.set(false);
        deployed = false;
    }

    public void stop() {
        winchMotor.stopMotor();
    }

    public boolean isDeployed() {
        return deployed;
    }
}