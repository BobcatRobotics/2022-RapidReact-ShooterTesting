package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    private WPI_TalonFX winchMotor;
    private DigitalInput leftWinchSwitch;
    private DigitalInput rightWinchSwitch;

    private Solenoid climberSolenoid;

    private boolean deployed = false;
    private boolean isClimberMode = false;

    private double climbMotorSpeedLimiter = 1.0;

    private double fullClimbingSpeed = 0.5;
    private double slowClimbingSpeed = 0.1;


    public Climber() {
        winchMotor = new WPI_TalonFX(winchMotorPort);
        
        winchMotor.configFactoryDefault();
        winchMotor.setNeutralMode(NeutralMode.Brake);
        winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        
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
        return leftWinchSwitch.get() || rightWinchSwitch.get();
    }

    public void climb(boolean isUnwinding) {
        // If limit switch has been hit, go slowly
        if (switchTripped()) {
            winchMotor.set(isUnwinding ? -slowClimbingSpeed : slowClimbingSpeed);
        }
        // Otherwise, go at normal speed
        else {
            winchMotor.set(isUnwinding ? -fullClimbingSpeed : fullClimbingSpeed);
        }
    }

    public void climb(Double climbSpeed) {
        winchMotor.set(climbSpeed * climbMotorSpeedLimiter);
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