package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class stopIntake extends CommandBase {

    Intake intake;

    public stopIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return !intake.isIntakeRunningIn();
    }
}
