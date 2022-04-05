package frc.robot.commands;

import javax.sound.midi.ControllerEventListener;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Ball;

public class alignToNearestBall extends CommandBase {

    private final Drivetrain m_drivetrain;
    private double angle = -1000;
    private double goal = -1000;
    private Ball ball = null;
    private int retries = 0;
    private PIDController controller;
    private double p = 1;
    private double i = .2;
    private boolean done = false;
    public alignToNearestBall(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ball = RobotContainer.getClosestBall();
        controller = new PIDController(p,i,0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(ball != null && angle != -1000){
            Double heading = Rotation2d.fromDegrees(m_drivetrain.getHeading()).getDegrees();
            // camera fov
            double degreeToTurn = (goal - heading) ;

            double turnPower = degreeToTurn / 27;
            if (degreeToTurn < .1) {
                degreeToTurn = 0.0;
                done = true;
            }
            
            turnPower = controller.calculate(turnPower,0);
            // double power = 12 * (degreeToTurn / angle) *.4;
            // System.out.println("degreeToTurn " + degreeToTurn+ "    heading " + heading );

            // if( Math.abs(power) < 2.5) {
            //     power = Math.signum(power) * 2.5;
            // }

            m_drivetrain.drive(-turnPower,turnPower);
            return;
        } 
        ball = RobotContainer.getClosestBall();
        if (ball != null) {
            System.out.println("got a ball");
            if(angle == -1000) {
            
                Double heading = Rotation2d.fromDegrees(m_drivetrain.getHeading()).getDegrees();

                angle = ball.getAngle();
                goal = heading + angle;
            }
        }  
        // m_drivetrain.tankDriveVolts(2.5,-2.5);   
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (done == true) {
            if (retries == 5) {
                System.out.println("aight imma head out");
                m_drivetrain.stop();
                return true;
            }
            ball = null;
            retries++;
        }
        return false;
    }
}
