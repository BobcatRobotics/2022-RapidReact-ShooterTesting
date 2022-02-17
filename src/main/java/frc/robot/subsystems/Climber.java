package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase
{
    private WPI_TalonFX winchMotor;
    private DigitalInput winchSwitch;

    private boolean deployed = false;

    private double climbMotorSpeedLimiter = 1.0;

    public Climber() {
        winchMotor = new WPI_TalonFX(winchMotorPort);
        
        winchMotor.configFactoryDefault();
        winchMotor.setNeutralMode(NeutralMode.Brake);
        winchMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
        winchSwitch = new DigitalInput(Constants.ClimberConstants.winchSwitchPos);
    }

    public void deploy() {
        deployed = true;
    }

    public boolean switchTripped() {
        return winchSwitch.get();
    }

    public void climb( Double climbSpeed) {
        winchMotor.set(climbSpeed * climbMotorSpeedLimiter);
    }

    public void withdraw() {
        deployed = false;
    }

    public void stop() {
        winchMotor.stopMotor();
    }

    public boolean getDeployed() {
        return deployed;
    }
}