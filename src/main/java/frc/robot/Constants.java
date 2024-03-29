/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* CONSTANTS INFO
   * From the official wpilib docs:
   * "It is recommended that users separate these constants 
   * into individual inner classes corresponding to subsystems or robot modes,
   * to keep variable names shorter."
   * "In Java, all constants should be declared public static final
   * so that they are globally accessible and cannot be changed."
   * "In Java, it is recommended that the constants be used from other classes
   * by statically importing the necessary inner class."
   */

  public static int[] desiredStatusFrames = {20, 20, 384, 384, 384, 384, 384, 384, 384, 384, 384, 384};
  public static StatusFrame[] frameTypes = {
    StatusFrame.Status_1_General,
    StatusFrame.Status_2_Feedback0,
    StatusFrame.Status_4_AinTempVbat,
    StatusFrame.Status_6_Misc,
    StatusFrame.Status_7_CommStatus,
    StatusFrame.Status_10_MotionMagic,
    StatusFrame.Status_10_Targets,
    StatusFrame.Status_12_Feedback1,
    StatusFrame.Status_13_Base_PIDF0,
    StatusFrame.Status_14_Turn_PIDF1,
    StatusFrame.Status_15_FirmwareApiStatus,
    StatusFrame.Status_17_Targets1,
  };
  public static int kTimeoutMs = 30;

  //Button definitions for gamepad
  public static final int A_Button = 2;
  public static final int B_Button = 3;
  public static final int X_Button = 1;
  public static final int Y_Button = 4;

  public static final int Right_Bumper_Button = 6;
  public static final int Right_Trigger_Button = 8;
  public static final int Left_Bumper_Button = 5;
  public static final int Left_Trigger_Button = 7;

  public static final int Left_Joystick_Pressed = 11;
  public static final int Right_Joystick_Pressed = 12;

  public static final int Gamepad_Left_Joystick_Y_Axis = 1;

  public static final int D_Pad_Up = 0;
  public static final int D_Pad_Down = 180;
  public static final int D_Pad_Left = 270;
  public static final int D_Pad_Right = 90;


  public static final String SHOOT_SCOOT = "Shoot & Scoot";
  public static final String SCOOT_N_SHOOT = "Scoot & Shoot";
  public static final String TRENCH_RUN = "Trench Run";
  public static final String PUSH = "PUSH";
  public static final String DONT_PUSH = "DONT_PUSH";

  public static final int RS_Shift_Switch = 3;

  public static final int LED_port = 0;
  
  /**
   * Constants for ball camera
   */
  public static final class BallCameraConstants {
    public static final double FRAME_WIDTH = 1280;
    public static final double FRAME_HEIGHT = 720;
    public static int cameraFOV = 27;
  }

  /**
   * Constants for the climber subsystem
   */
   
  public static final class ClimberConstants
  {
    public static final int winchMotorPort = 42; // DONT MAKE THIS THE SAME AS SHOOTER PLS
    public static final int climberSolenoidPort = 2;
    public static final int leftWinchSwitchPort = 0;
    public static final int rightWinchSwitchPort = 1;
  }

   /**
    * Constants for the drive train
    */
  public static final class DrivetrainConstants {

    public static final double INVERT_MOTOR = -1.0;
    // Drive Train motors
    public static final int LTMotorPort = 0;
    public static final int LMMotorPort = 1;
    public static final int LLMotorPort = 2;
    public static final int RTMotorPort = 3;
    public static final int RMMotorPort = 4; 
    public static final int RLMotorPort = 5;

    // Need to edit all the numbers under this for 2020
    public static final int[] leftEncoderPorts = new int[] { 0, 1 };
    public static final int[] rightEncoderPorts = new int[] { 3, 4 };
    public static final boolean leftEncoderReversed = false;
    public static final boolean rightEncoderReversed = false;

    public static final int encoderCPR = 128;
    public static final double wheelDiameterMeters = 0.1;
    public static final double encoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (wheelDiameterMeters * Math.PI) / (double) encoderCPR;

    public static final boolean GYRO_REVERSED = true;
  }

  /**
   * Constants for the feeder system
   */
  public static final class FeederConstants {
    public static final double INVERT_MOTOR = -1.0;

    // Ball Present Sensor Config
    public static final int feederBallPresentId = 0;
    public static final int feederBallLeavingId = 1; //TODO: change urgently or else we all die
    public static final double feederBallPresentThreshold = 125.0;
    public static final double feederBallLeavingThreshold = 125.0; // TODO: change to correct value

    // Makes sure the speed does not increase over this number
    public static final double speedLimiter = -0.7;

    public static final double speedLimiterSlow = -0.3;


    // compressor config
    public static final int feederCompressorModule = 1; // TODO: change to correct value
  }

  /**
   * Constants for the intake system
   */
  public static final class IntakeConstants {
    // Intake motors
    public static final int intakeLeftWheelPort = 13;
    public static final int intakeRightWheelPort = 7;
    public static final int intakeBarPort = 6;

    // Intake solenoids
    public static final int intakeSolenoidPort = 0;
  }

  /**
   * Constants for the input device ports
   */
  public static final int compressorModelPort = 1;

  public static final class RobotContainerConstants {
    // Sticks
    public static final int leftStickPort = 0;
    public static final int rightStickPort = 1;
    public static final int gamepadPort = 2;

    // Gamepad POV values in degrees
    public static final int povUp = 0;
    public static final int povRight = 90;
    public static final int povDown = 180;
    public static final int povLeft = 270;
  }
  
  /**
   * Constants for path finding and trajectories
   */
  public static final class RouteFinderConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these values for your robot.
    public static final double ksVolts = 1.43;
    public static final double kvVoltSecondsPerMeter = 5.15;
    public static final double kaVoltSecondsSquaredPerMeter = 0.384;
    public static final double kTrackwidthMeters = 0.77;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = -12.5;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // On-the-go route finding waypoint variables
    public static final int pointx = 0;
    public static final int pointy = 0;
    public static final int rotation = 0;
  }
  
  /**
   * Constants for the shooter subsystem
   */
  public static final class ShooterConstants {
    // Shooter spinner motors
    public static final int DEFAULT_LOWER_HUB_SHOOTING_SPEED = 1800;
    public static final int DEFAULT_UPPER_HUB_SHOOTING_SPEED = 2400;
    public static final int shooterFalcon1Port = 8; //changed from 8 in Phoenix tuner
    public static final int shooterFalcon2Port = 9;
    public static final int hoodFalconPort = 15;
    public static final int feedMotorPort = 10;
    public static final int shooterAngleSolenoidPort = 1;
    public static final int tofPort = 17;
    public static final int defaultTofRange = 150;
    public static final double UPPER_HUB_KEY = 0.0;
    public static final double LOWER_HUB_KEY = 0.5;
    public static HashMap<Double, double[]> LIMELIGHT_SHOOTING_LOOKUP_MAP = new HashMap<Double, double[]>();
    static { // Key: dist (m). Val: [main flywheel, hood wheel]

      // --- Manual upper hub shooting
      
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(0.0, new double[]{2480, 2340});
      
      // --- Manual lower hub shooting
      
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(0.5, new double[]{300, 1300});
      
      // --- Limelight-based shooting

      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(1.0, new double[]{2200, 1800});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(1.25, new double[]{2200, 1800});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(1.5, new double[]{2200, 1800});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(1.75, new double[]{2200, 1900});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(2.0, new double[]{2100, 2050});

      // --- Below are tested values; above are predicted

      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(2.25, new double[]{2100, 2100});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(2.5, new double[]{2375, 2275});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(2.75, new double[]{2375, 2275});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(3.0, new double[]{2375, 2275});
      // LIMELIGHT_SHOOTING_LOOKUP_MAP.put(3.25, new double[]{2475, 2345});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(3.25, new double[]{2450, 2325});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(3.5, new double[]{2510, 2390});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(3.75, new double[]{2530, 2470});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(4.0, new double[]{2605, 2555});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(4.25, new double[]{2660, 2605});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(4.5, new double[]{2735, 2655});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(4.75, new double[]{2770, 2700});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(5.0, new double[]{2825, 2765});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(5.25, new double[]{2830, 2795});

      // --- Above are tested values; below are predicted

      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(5.5, new double[]{2890, 3083});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(5.75, new double[]{2927, 3208});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(6.0, new double[]{2962, 3342});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(6.25, new double[]{2994, 3483});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(6.5, new double[]{3025, 3633});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(6.75, new double[]{3053, 3791});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(7.0, new double[]{3079, 3957});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(7.25, new double[]{3102, 4131});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(7.5, new double[]{3123, 4314});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(7.75, new double[]{3142, 4504});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(8.0, new double[]{3159, 4702});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(8.25, new double[]{3174, 4904});

      // --- Below are tested values; above are predicted

      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(8.5, new double[]{3200, 5500});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(8.75, new double[]{3200, 5500});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(9.0, new double[]{3200, 5500});
      LIMELIGHT_SHOOTING_LOOKUP_MAP.put(9.25, new double[]{3200, 5500});
    }
  }

  public static final class LimelightConstants {
    public static final double kLimelightHeight = Units.inchesToMeters(104) - Units.inchesToMeters(27.375); // hub height - limelight mount height, meters
    // public static final double kLimelightHeight = Units.feetToMeters(6) - Units.inchesToMeters(63); // hub height - limelight mount height, meters
    public static final double kLimelightMountAngle = 36 * (Math.PI/180); // rad
  }

  public static final class StopAtCollisionConstants {
      public static boolean collision = true;
      public static final double kCollisionThreshold_DeltaG = 1.2; 
  }
}
