package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {

    private Drivetrain drivetrain;
    private Joystick throttleStick;
    private Joystick steerStick;
    
    public ArcadeDrive(Drivetrain drivetrain, Joystick rightStickSteer, Joystick leftStickThrottle) {
        this.drivetrain = drivetrain;
        this.throttleStick = leftStickThrottle;
        this.steerStick = rightStickSteer;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double throttle = throttleStick.getRawAxis(Joystick.AxisType.kY.value);
        double steer = steerStick.getRawAxis(Joystick.AxisType.kX.value);
        if (Math.abs(throttle) < 0.07) throttle = 0;
        if (Math.abs(steer) < 0.07) steer = 0;
        double max = Double.max(Math.abs(throttle), Math.abs(steer));
        double total = throttle + steer, diff = throttle - steer;
        drivetrain.drive(
            throttle >= 0 ? (steer >= 0 ? diff : max) : (steer >= 0 ? -max : diff),
            throttle >= 0 ? (steer >= 0 ? max : total) : (steer >= 0 ? total : -max),
            false
        );
    }

}
