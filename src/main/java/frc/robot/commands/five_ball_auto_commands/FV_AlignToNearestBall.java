package frc.robot.commands.five_ball_auto_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Ball;

public class FV_AlignToNearestBall extends CommandBase {

    private final Drivetrain m_drivetrain;
    private double angle = -1000;
    private double goal = -1000;
    private Ball ball = null;
    private int retries = 0;
    private PIDController controller;
    private double p = 0.75;
    private double i = 0.05;
    private boolean done = false;
    private double BigDegrees = -1000;
    private Timer timeout;
    private boolean timerHasNotStarted = true;
    private double limit = 0.1;
    public FV_AlignToNearestBall(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        timeout = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ball = RobotContainer.getClosestBall();
        System.out.println("Starting center on ball");
        controller = new PIDController(p,i,0);
        // timeout.reset();
        // timeout.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(ball != null && angle != -1000){
            if (timerHasNotStarted) {
                timeout.reset();
                timeout.start();
                timerHasNotStarted = false;
            }
            Double heading = Rotation2d.fromDegrees(m_drivetrain.getHeading()).getDegrees();
            // camera fov
            double degreeToTurn = (goal - heading) % 360 ;
            System.out.println(degreeToTurn);
            double turnPower = degreeToTurn / 27;
            // if (Math.abs(degreeToTurn) < limit) {
            //     degreeToTurn = 0.0;
            //     turnPower = 0.0;
            //     done = true;
            // }
            
            // turnPower = controller.calculate(turnPower,0);
            // double power = 12 * (degreeToTurn / angle) *.4;
            // System.out.println("degreeToTurn " + degreeToTurn+ "    heading " + heading );

            // if( Math.abs(power) < 2.5) {
            //     power = Math.signum(power) * 2.5;
            // }
            double d = controller.calculate(turnPower,0);
            BigDegrees = d;
            if (d > 0.3) d = 0.3;
            else if (d < -0.3) d = -0.3;
            // if (0 < d && d < 0.08) d = 0.08;
            // else if (0 > d && d > -0.08) d = -0.08;
            // if (Math.abs(degreeToTurn) >= limit) {
            //     timeout.stop();
            //     timeout.reset();
            //     timeout.start();
            // }
            m_drivetrain.drive(d, -d, false);

            /*
            System.out.println("PID  " + d);
            if (Math.abs(turnPower) >= 1) {
                turnPower = 1 * Math.signum(turnPower);
            }
            turnPower*=.5;
            if (Math.abs(turnPower) < .075 && turnPower != 0.0) {
                turnPower = .1 * Math.signum(turnPower);
            }

            System.out.println("Turn Power" +turnPower);

            m_drivetrain.drive(-turnPower,turnPower, false);
            */

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

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isDone = timeout.hasElapsed(0.75);
        if (isDone) System.out.println("DONE DONE DONE YA YEET");
        return isDone;
        // if (done == true) {
        // if (Math.abs(BigDegrees) < 0.1) {
        //     if (retries == 15) {
        //         System.out.println("aight imma head out");
        //         m_drivetrain.stop();
        //         return true;
        //     }
        //     ball = null;
        //     retries++;
        // }
        // return false;
    }
}
