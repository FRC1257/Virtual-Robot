// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Elevator {
    public static class ElevatorSimConstants {
      public static final int kMotorPort = 0;
      public static final int kEncoderAChannel = 0;
      public static final int kEncoderBChannel = 1;
      public static final int kJoystickPort = 0;

      public static final double kElevatorKp = 5;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0;

      public static final double kElevatorkS = 0.0; // volts (V)
      public static final double kElevatorkG = 0.762; // volts (V)
      public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
      public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

      public static final double kElevatorGearing = 10.0;
      public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
      public static final double kCarriageMass = 4.0; // kg

      public static final double kSetpointMeters = 0.75;
      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinElevatorHeightMeters = 0.0;
      public static final double kMaxElevatorHeightMeters = 2.25;

      // distance per pulse = (distance per revolution) / (pulses per revolution)
      // = (Pi * D) / ppr
      public static final double kElevatorEncoderDistPerPulse = 2.0 * Math.PI * kElevatorDrumRadius / 4096;
    }

    public static class ElevatorPhysicalConstants {

      public static final double[] ELEVATOR_PID = new double[] { 0.1, 0, 0, 0 };

      // TODO Tune
      public static double ELEVATOR_PID_MAX_OUTPUT = 0.7;
      public static double ELEVATOR_REV_TO_POS_FACTOR = 1;

      public static double ELEVATOR_PID_TOLERANCE = 3;

      // TODO Tune
      public static final double ELEVATOR_SETPOINT_RETRACT = 0;
      public static final double ELEVATOR_SETPOINT_EXTEND = 32;

      public static final double ELEVATOR_SETPOINT_MIDDLE = (ELEVATOR_SETPOINT_RETRACT + ELEVATOR_SETPOINT_EXTEND) / 2;

      public static final double ELEVATOR_STOP_BUFFER = 5;
    }

    public static final double ELEVATOR_TOLERANCE = 3;

  }

  public static class Laterator {
    public static class LateratorSimConstants {
      public static final int kMotorPort = 0;
      public static final int kEncoderAChannel = 0;
      public static final int kEncoderBChannel = 1;
      public static final int kJoystickPort = 0;

      public static final double kLateratorKp = 5;
      public static final double kLateratorKi = 0;
      public static final double kLateratorKd = 0;

      public static final double kLateratorS = 0.0; // volts (V)
      public static final double kLateratorG = 0.762; // volts (V)
      public static final double kLateratorV = 0.762; // volt per velocity (V/(m/s))
      public static final double kLateratorA = 0.0; // volt per acceleration (V/(m/s²))

      public static final double kLateratorGearing = 10.0;
      public static final double kLateratorDrumRadius = Units.inchesToMeters(2.0);
      public static final double kCarriageMass = 4.0; // kg

      public static final double kSetpointMeters = 0.75;
      // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
      public static final double kMinLateratorHeightMeters = 0.0;
      public static final double kMaxLateratorHeightMeters = 2.25;

      // distance per pulse = (distance per revolution) / (pulses per revolution)
      // = (Pi * D) / ppr
      public static final double kLateratorEncoderDistPerPulse = 2.0 * Math.PI * kLateratorDrumRadius / 4096;
    }

    public static class LateratorPhysicalConstants {

      public static final double[] LATERATOR_PID = new double[] { 0.1, 0, 0, 0 };

      // TODO Tune
      public static double LATERATOR_PID_MAX_OUTPUT = 0.7;
      public static double LATERATOR_REV_TO_POS_FACTOR = 1;

      public static double LATERATOR_PID_TOLERANCE = 3;

      // TODO Tune
      public static final double LATERATOR_SETPOINT_RETRACT = 0;
      public static final double LATERATOR_SETPOINT_EXTEND = 32;

      public static final double LATERATOR_SETPOINT_MIDDLE = (LATERATOR_SETPOINT_RETRACT + LATERATOR_SETPOINT_EXTEND) / 2;

      public static final double LATERATOR_STOP_BUFFER = 5;
    }

    public static final double LATERATOR_TOLERANCE = 3;

  }

  public static class PivotArm {
    // PID constants
    public static double[] PIVOT_ARM_PID = new double[] { 0.25, 0, 0, 0 };
    public static double PIVOT_ARM_PID_TOLERANCE = 1;
    public static double PIVOT_ARM_PID_MAX_OUTPUT = 1;

    public static double POSITION_CONVERSION_FACTOR = 1;

    // Setpoints between -1 and 1
    public static double PIVOT_ARM_SETPOINT_UP = 135;
    public static double PIVOT_ARM_SETPOINT_MID = 175;
    // public static double PIVOT_ARM_SETPOINT_INTAKE = 0; // also used for low
    // score
    public static double PIVOT_ARM_SETPOINT_HOLD = 10;
    public static final double PIVOT_ARM_SETPOINT_BOTTOM = 0;
    public static final double PIVOT_ARM_SETPOINT_TOP = 170;

    public static class PivotArmPhysicalConstants {
      public static final double PIVOT_ARM_TOLERANCE = 3;
      public static final double PIVOT_ARM_STOP_BUFFER = 5;
    }

    public static class PivotArmSimConstants {
      public static final int kMotorPort = 2;
      public static final int kEncoderAChannel = 2;
      public static final int kEncoderBChannel = 3;
      public static final int kJoystickPort = 0;

      public static final String kArmPositionKey = "ArmPosition";
      public static final String kArmPKey = "ArmP";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmKp = 50.0;
      public static final double kDefaultArmSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      // = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 200;
      public static final double kArmMass = 5.0; // Kilograms
      public static final double kArmLength = Units.inchesToMeters(20);
      public static final double kMinAngleRads = Units.degreesToRadians(-175);
      public static final double kMaxAngleRads = Units.degreesToRadians(255);
      public static double kEncoderDistancePerPulse;
    }
  }

  public static class PivotWrist {
    // PID constants
    public static double[] PIVOT_WRIST_PID = new double[] { 0.25, 0, 0, 0 };
    public static double PIVOT_WRIST_PID_TOLERANCE = 1;
    public static double PIVOT_WRIST_PID_MAX_OUTPUT = 1;

    public static double POSITION_CONVERSION_FACTOR = 1;

    // Setpoints between -1 and 1
    public static double PIVOT_WRIST_SETPOINT_UP = 135;
    public static double PIVOT_WRIST_SETPOINT_MID = 175;
    // public static double PIVOT_WRIST_SETPOINT_INTAKE = 0; // also used for low
    // score
    public static double PIVOT_WRIST_SETPOINT_HOLD = 10;
    public static final double PIVOT_WRIST_SETPOINT_BOTTOM = 0;
    public static final double PIVOT_WRIST_SETPOINT_TOP = 170;

    public static class PivotWristPhysicalConstants {
      public static final double PIVOT_WRIST_TOLERANCE = 3;
      public static final double PIVOT_WRIST_STOP_BUFFER = 5;
    }

    public static class PivotWristSimConstants {
      public static final int kMotorPort = 2;
      public static final int kEncoderAChannel = 2;
      public static final int kEncoderBChannel = 3;
      public static final int kJoystickPort = 0;

      public static final String kWristPositionKey = "WristPosition";
      public static final String kWristPKey = "WristP";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultWristKp = 50.0;
      public static final double kDefaultWristSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      // = (2 * PI rads) / (4096 pulses)
      public static final double kWristEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kWristReduction = 200;
      public static final double kWristMass = 5.0; // Kilograms
      public static final double kWristLength = Units.inchesToMeters(20);
      public static final double kMinAngleRads = Units.degreesToRadians(-175);
      public static final double kMaxAngleRads = Units.degreesToRadians(255);
      public static double kEncoderDistancePerPulse;
    }
  }

  public static class IntakeArm {
    // PID constants
    public static double[] INTAKE_ARM_PID = new double[] { 0.25, 0, 0, 0 };
    public static double INTAKE_ARM_PID_TOLERANCE = 1;
    public static double INTAKE_ARM_PID_MAX_OUTPUT = 1;

    public static double POSITION_CONVERSION_FACTOR = 1;

    // Setpoints between -1 and 1
    public static double INTAKE_ARM_SETPOINT_UP = 135;
    public static double INTAKE_ARM_SETPOINT_MID = 175;
    // public static double INTAKE_ARM_SETPOINT_INTAKE = 0; // also used for low
    // score
    public static double INTAKE_ARM_SETPOINT_HOLD = 10;
    public static final double INTAKE_ARM_SETPOINT_BOTTOM = 0;
    public static final double INTAKE_ARM_SETPOINT_TOP = 170;

    public static class IntakeArmPhysicalConstants {
      public static final double INTAKE_ARM_TOLERANCE = 3;
      public static final double INTAKE_ARM_STOP_BUFFER = 5;
    }

    public static class IntakeArmSimConstants {
      public static final int kMotorPort = 2;
      public static final int kEncoderAChannel = 2;
      public static final int kEncoderBChannel = 3;
      public static final int kJoystickPort = 0;

      public static final String kArmPositionKey = "ArmPosition";
      public static final String kArmPKey = "ArmP";

      // The P gain for the PID controller that drives this arm.
      public static final double kDefaultArmKp = 50.0;
      public static final double kDefaultArmSetpointDegrees = 75.0;

      // distance per pulse = (angle per revolution) / (pulses per revolution)
      // = (2 * PI rads) / (4096 pulses)
      public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

      public static final double kArmReduction = 200;
      public static final double kArmMass = 5.0; // Kilograms
      public static final double kArmLength = Units.inchesToMeters(20);
      public static final double kMinAngleRads = Units.degreesToRadians(-175);
      public static final double kMaxAngleRads = Units.degreesToRadians(255);
      public static double kEncoderDistancePerPulse;
    }
  }  

  public static class Claw {
    /* public static double[] CLAW_PID = new double[] {0, 0, 0, 0};
    public static double CLAW_PID_TOLERANCE = 0.1257; // lol
    public static double CLAW_PID_MAX_OUTPUT = 0.1257; // lol */

    public static double CLAW_CLOSED_SPEED = 0.5;
    public static double CLAW_OPEN_SPEED = -0.5;

    public static double POSITION_CONVERSION_FACTOR = 1;

    public static double CLAW_OPEN_TIME = 2.0;
    public static double CLAW_CLOSE_TIME = 2.20;

    public static class ClawSimConstants {
      public static final int kEncoderAChannel = 4;
      public static final int kEncoderBChannel = 5;
      public static final double kEncoderDistancePerPulse = 3;

    }
};

public static class Intake {
  public static class IntakeSimConstants {
    public static final int kMotorPort = 3;
    public static final int[] kEncoderPorts = new int[] { 6, 7 };
  }

  public static double POSITION_CONVERSION_FACTOR = 0.5;
}

  public static class Drivetrain {
    // drivetrain constants
    public static double DRIVE_TRACK_WIDTH_M = 0.86;// 0.66; // m
    public static double DRIVE_WHEEL_DIAM_M = 0.1524; // m
    public static double DRIVE_GEARBOX_REDUCTION = 10.71;

    // driving modifiers
    public static double DRIVE_SLOW_TURN_MULT = 0.25;
    public static double DRIVE_SLOW_FORWARD_MULT = 0.25;

    // closed loop driving
    public static double DRIVE_CLOSED_MAX_VEL = 4.0; // m/s
    public static double DRIVE_CLOSED_MAX_ROT_TELEOP = 360.00; //
    public static double DRIVE_CLOSED_MAX_ROT_AUTO = 100.0; // deg/s
    public static double DRIVE_CLOSED_MAX_ACC = 1.5; // m/s^2

    // trajectory following
    public static double DRIVE_TRAJ_MAX_VEL = 8.0; // m/s
    public static double DRIVE_TRAJ_MAX_ACC = 0.7515; // .75; // m/s^2
    public static double DRIVE_TRAJ_RAMSETE_B = 2.0; // don't change
    public static double DRIVE_TRAJ_RAMSETE_ZETA = 0.7;

    public static double DRIVE_TRAJ_KV = 0.0; // don't change
    public static double DRIVE_TRAJ_KA = 0.0; // don't change
    public static double DRIVE_TRAJ_KS = 0.0; // don't change

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // aligning
    public static double DRIVE_ALIGN_MAX_VEL = 0.75; // m/s
    public static double DRIVE_ALIGN_MAX_ACC = 0.350; // .75; // m/s^2

    // linear position PID
    public static double[] DRIVE_DIST_PID = { 3.50, 0.0, 0.0 };
    public static double DRIVE_DIST_ANGLE_P = 0.1;
    public static double DRIVE_DIST_TOLERANCE = 0.01;
    public static double DRIVE_DIST_MAX_OUTPUT = 0.6;

    // angular position PID works for test bot
    public static double[] DRIVE_ANGLE_PID = { 0.045, 0.1, 0.005 }; // 0.055
    public static double DRIVE_ANGLE_TOLERANCE = 0.5;
    public static double DRIVE_ANGLE_MAX_OUTPUT = 0.6;

    // velocity PID (for closed loop, profiling, and trajectory)
    public static int DRIVE_VEL_SLOT = 0;
    public static double DRIVE_VEL_LEFT_P = 0.25;
    public static double DRIVE_VEL_LEFT_F = 0.25;
    public static double DRIVE_VEL_RIGHT_P = 0.25;
    public static double DRIVE_VEL_RIGHT_F = 0.25;

    // profiling position PID (for further refinement of tracking)
    public static double DRIVE_PROFILE_LEFT_P = 0.1;
    public static double DRIVE_PROFILE_RIGHT_P = 0.1;

    // vision PID
    public static final double TRACKED_TAG_ROTATION_KP = 0.375;
    public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2; // P (Proportional) constant of a PID loop
    public static final double APRILTAG_POWER_CAP = 0.75;
  };

  public static class ElectricalLayout {
    // Controllers
    public final static int CONTROLLER_DRIVER_ID = 0;
    public final static int CONTROLLER_OPERATOR_ID = 1;

    // Drivetrain Test Bot
    /*
     * public final static int DRIVE_FRONT_LEFT = 17;
     * public final static int DRIVE_FRONT_RIGHT = 10;
     * public final static int DRIVE_BACK_LEFT = 9;
     * public final static int DRIVE_BACK_RIGHT = 11;
     */

    // Drivetrain Main
    public final static int DRIVE_FRONT_LEFT = 1;
    public final static int DRIVE_FRONT_RIGHT = 2;
    public final static int DRIVE_BACK_LEFT = 3;
    public final static int DRIVE_BACK_RIGHT = 4;

    // Intakes
    public final static int INTAKE_MOTOR_ID = 9;
    public final static int INTAKE_ARM_MOTOR_ID = 6;

    /* public static final int INTAKE_ARM_MOTOR_LEFT_ID = 6;
    public static final int INTAKE_ARM_MOTOR_RIGHT_ID = 8; */

    // Claw
    public final static int CLAW_MOTOR_LEFT_ID = 10;
    public final static int CLAW_PIECE_BUTTON = 0;

    // New Elevator Motor Design
    public final static int ELEVATOR_MOTOR_ID = 5;
    public static final int LATERATOR_MOTOR_ID = 11;

    // Pivot ARm
    public static int PIVOT_ARM_ID = 7;
    public static int PIVOT_WRIST_ID = 12;

    // Sensors
    public static final int INTAKE_BUMP_SWITCH_ID = 24;
    public static final int ELEVATOR_LIMIT_SWITCH = 0;
  };

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public final static int NEO_550_CURRENT_LIMIT = 25; // amps
  public final static int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  /** Ambiguous with NEO_CURRENT_LIMIT in ElectricalLayout */
  // public final static int NEO_CURRENT_LIMIT = 80; // amps

  public final static int NEO_CURRENT_LIMIT = 80; // amps
}
