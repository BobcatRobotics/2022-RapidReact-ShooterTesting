package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTele extends CommandBase {
    // If the speed of a motor is less than this threshold, we'll just set it to zero
    // private final double driveThreshold = Math.sqrt(0.07);
    private final double driveThreshold = 0.07;

    // Represents the position of the right and left joysticks
    // These determine the speed of the right and left motors
    private double right = 0.0;
    private double left = 0.0;

    private final Drivetrain drivetrain;
    private final Joystick rightStick;
    private final Joystick leftStick;

    public DriveTele(Drivetrain drivetrain, Joystick rightStick, Joystick leftStick) {
        this.drivetrain = drivetrain;
        this.rightStick = rightStick;
        this.leftStick = leftStick;
        addRequirements(drivetrain);
    }

    /**
     * Sets the motor speed values to the values from the right and left joysticks
     * It can only drive if it is teleoperated mode.
     */
    @Override
    public void execute() {
        if (RobotState.isOperatorControl()) {
        // Gets the values of the right and left joystick positions
        right = rightStick.getRawAxis(Joystick.AxisType.kY.value);
        left = leftStick.getRawAxis(Joystick.AxisType.kY.value);

        // If the value of the joysticks are too low just set it to zero
        if (Math.abs(right) <= driveThreshold) {
            right = 0.0;
        }

        if (Math.abs(left) <= driveThreshold) {
            left = 0.0;
        }

        drivetrain.drive(right, left, false);
        } else {
            drivetrain.stop();
        }
    }

    // Runs when command is interrupted
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
