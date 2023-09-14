package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FieldConstants {
    static double trackWidthAdded = Constants.Drivetrain.DRIVE_TRACK_WIDTH_M / 2;
    public static double BALANCE_KP = 0.05;
    public static double BALANCE_KD = 0.01;
    public static double BALANCE_SETPOINT_ANGLE = 0;
    public static double BALANCE_THRESHOLD_DEGREES = 3;
    public static int BALANCE_STEPS_THRESHOLD = 25;

    public static double BOTTOM_PICKUP_Y = feet(20);
    public static double TOP_PICKUP_Y = feet(24.6);

    public static double CHARGE_STATION_LOWER_Y = 1.508506;
    public static double CHARGE_STATION_UPPER_Y = 3.978656;

    public static final double BLUE_COMMUNITY_X = 3.34;
    public static final double RED_COMMUNITY_X = 13.2;

    public static double CHARGE_CENTER_Y = 2.75;

    public static double feet(double feet) {
        // convert feet to meters
        return Units.feetToMeters(feet);
    }

    public static Pose2d[] BLUE_SCORE_POSE = new Pose2d[] {
        new Pose2d(1.425 + trackWidthAdded, 0.453, Rotation2d.fromDegrees(180)), // Score location 1 on
        // blue side
        new Pose2d(1.425 + trackWidthAdded, 1.044, Rotation2d.fromDegrees(180)), // score 2
        new Pose2d(1.425 + trackWidthAdded, 1.579, Rotation2d.fromDegrees(180)), // 3
        new Pose2d(1.425 + trackWidthAdded, 2.204, Rotation2d.fromDegrees(180)), // ...
        new Pose2d(1.425 + trackWidthAdded, 2.773, Rotation2d.fromDegrees(180)),
        new Pose2d(1.425 + trackWidthAdded, 3.273, Rotation2d.fromDegrees(180)),
        new Pose2d(1.425 + trackWidthAdded, 3.831, Rotation2d.fromDegrees(180)),
        new Pose2d(1.425 + trackWidthAdded, 4.433, Rotation2d.fromDegrees(180)),
        new Pose2d(1.425 + trackWidthAdded, 5.082, Rotation2d.fromDegrees(180))
    };

    public static Pose2d[] RED_SCORE_POSE = new Pose2d[] {
        new Pose2d(15.15 - trackWidthAdded, 0.453, Rotation2d.fromDegrees(0)), // Score location 1 on
        // blue side
        new Pose2d(15.15 - trackWidthAdded, 1.044, Rotation2d.fromDegrees(0)), // score 2
        new Pose2d(15.15 - trackWidthAdded, 1.579, Rotation2d.fromDegrees(0)), // 3
        new Pose2d(15.15 - trackWidthAdded, 2.204, Rotation2d.fromDegrees(0)), // ...
        new Pose2d(15.15 - trackWidthAdded, 2.773, Rotation2d.fromDegrees(0)),
        new Pose2d(15.15 - trackWidthAdded, 3.273, Rotation2d.fromDegrees(0)),
        new Pose2d(15.15 - trackWidthAdded, 3.831, Rotation2d.fromDegrees(0)),
        new Pose2d(15.15 - trackWidthAdded, 4.433, Rotation2d.fromDegrees(0)),
        new Pose2d(15.15 - trackWidthAdded, 5.082, Rotation2d.fromDegrees(0))
    };

    public static Pose2d[] BLUE_CARGO_POSE = new Pose2d[] {
        new Pose2d(7.066, 0.896, Rotation2d.fromDegrees(0)),
        new Pose2d(7.066, 2.125, Rotation2d.fromDegrees(0)),
        new Pose2d(7.066, 3.353, Rotation2d.fromDegrees(0)),
        new Pose2d(7.066, 4.602, Rotation2d.fromDegrees(0)),
    };

    public static Pose2d[] RED_CARGO_POSE = new Pose2d[] {
        new Pose2d(9.5, 0.896, Rotation2d.fromDegrees(180)),
        new Pose2d(9.5, 2.125, Rotation2d.fromDegrees(180)),
        new Pose2d(9.5, 3.353, Rotation2d.fromDegrees(180)),
        new Pose2d(9.5, 4.602, Rotation2d.fromDegrees(180)),
    };

    public static Pose2d[] BLUE_WAYPOINT_POSE = new Pose2d[] {
        new Pose2d(2.87, 4.73, Rotation2d.fromDegrees(0)),
        new Pose2d(4.8, 4.73, Rotation2d.fromDegrees(0)),
        new Pose2d(2.87, 0.754, Rotation2d.fromDegrees(0)),
        new Pose2d(4.8, 0.754, Rotation2d.fromDegrees(0)),
    };
    public static Pose2d[] RED_WAYPOINT_POSE = new Pose2d[] {
        new Pose2d(13.5, 4.73, Rotation2d.fromDegrees(180)),
        new Pose2d(11.6, 4.73, Rotation2d.fromDegrees(180)),
        new Pose2d(13.5, 0.754, Rotation2d.fromDegrees(180)),
        new Pose2d(11.6, 0.754, Rotation2d.fromDegrees(180)),
    };

    public static Pose2d BLUE_CHARGE_POSE[] = new Pose2d[] {
        new Pose2d(3.89, 2.75, Rotation2d.fromDegrees(180)),
        new Pose2d(3.89, 2.75, Rotation2d.fromDegrees(0)),
    };
    public static Pose2d[] RED_CHARGE_POSE = new Pose2d[] {
        new Pose2d(12.58, 2.75, Rotation2d.fromDegrees(0)),
        new Pose2d(12.58, 2.75, Rotation2d.fromDegrees(180)),
    };

    public static Pose2d[] BLUE_CHARGE_POSE_WAYPOINT = new Pose2d[] {
        new Pose2d(5.4, 2.75, Rotation2d.fromDegrees(180)),
        new Pose2d(3.2, 2.75, Rotation2d.fromDegrees(0)),
    };
    public static Pose2d[] RED_CHARGE_POSE_WAYPOINT = new Pose2d[] {
        new Pose2d(11, 2.75, Rotation2d.fromDegrees(0)),
        new Pose2d(13.5, 2.75, Rotation2d.fromDegrees(180)),
    };

    public static Pose2d[] BLUE_LEAVE_COMMUNITY_POSE = new Pose2d[] {
        new Pose2d(5, 4.73, Rotation2d.fromDegrees(0)),
        new Pose2d(5, 0.754, Rotation2d.fromDegrees(0)),
        // this y coord is aligned correctly i think, it's a little jank
        new Pose2d(5.5, BLUE_SCORE_POSE[4].getY(), Rotation2d.fromDegrees(0)),
    };

    public static Pose2d[] RED_LEAVE_COMMUNITY_POSE = new Pose2d[] {
        new Pose2d(11.4, 4.73, Rotation2d.fromDegrees(180)),
        new Pose2d(11.4, 0.754, Rotation2d.fromDegrees(180)),
        new Pose2d(10.7, RED_SCORE_POSE[4].getY(), Rotation2d.fromDegrees(180)),
    };

    // Positions for piece pickup
    public static Pose2d[] BLUE_PICKUP = new Pose2d[] {
        new Pose2d(feet(53) - trackWidthAdded, BOTTOM_PICKUP_Y, Rotation2d.fromDegrees(0)),
        new Pose2d(feet(53) - trackWidthAdded, TOP_PICKUP_Y, Rotation2d.fromDegrees(0)),
        new Pose2d(feet(46.8), feet(26.3) - trackWidthAdded, Rotation2d.fromDegrees(90)),
    };

    public static Pose2d[] RED_PICKUP = new Pose2d[] {
        new Pose2d(feet(1.24) + trackWidthAdded, BOTTOM_PICKUP_Y, Rotation2d.fromDegrees(180)),
        new Pose2d(feet(1.24) + trackWidthAdded, TOP_PICKUP_Y, Rotation2d.fromDegrees(180)),
        new Pose2d(feet(7.77), feet(26.3) - trackWidthAdded, Rotation2d.fromDegrees(270)),
    };

    // Waypoints for trajectories
    public static Translation2d BLUE_SUBSTATION_ENTRY = new Translation2d(feet(53) - trackWidthAdded, feet(26.3) - trackWidthAdded);
    public static Translation2d RED_SUBSTATION_ENTRY = new Translation2d(feet(16.056), feet(21.56));
    
    public static Translation2d[][] BLUE_WAYPOINTS = new Translation2d[][] {
        // Higher Path
        {
            new Translation2d(feet(8.444), feet(15.441)),
            new Translation2d(feet(18.78), feet(15.441)),
        },
        // Across Charge Station
        {
            new Translation2d(feet(8.892), CHARGE_CENTER_Y),
            new Translation2d(feet(18.78), CHARGE_CENTER_Y),
        },
        // Lower Path
        {
            new Translation2d(feet(8.444), feet(2.455)),
            new Translation2d(feet(19.564), feet(2.604)),
        }
    };
    public static Translation2d[][] RED_WAYPOINTS = new Translation2d[][]{
        // Higher Path
        {
            new Translation2d(feet(44.6), feet(15.1)),
            new Translation2d(feet(35.98), feet(15.36)),
        },
        // Across Charge Station
        {
            new Translation2d(feet(45.386), feet(9.172)),
            new Translation2d(feet(36.057), feet(9.172)),
        },
        // Lower Path
        {
            new Translation2d(feet(45.647), feet(2.567)),
            new Translation2d(feet(35.348), feet(2.567)),
        }
    };

    public static Pose2d[] BLUE_PARK_POSE = new Pose2d[] {
        new Pose2d(2.324, 4.593, Rotation2d.fromDegrees(0)),
        new Pose2d(4, 0.716, Rotation2d.fromDegrees(0)),
    };

    public static Pose2d[] RED_PARK_POSE = new Pose2d[] {
        new Pose2d(13.847, 4.593, Rotation2d.fromDegrees(180)),
        new Pose2d(12.57, 0.716, Rotation2d.fromDegrees(180)),
    };
}
