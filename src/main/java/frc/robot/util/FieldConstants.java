package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class FieldConstants {
    static double trackWidthAdded = Constants.Drivetrain.DRIVE_TRACK_WIDTH_M / 2;
    public static double BALANCE_KP = 0.05;
    public static double BALANCE_KD = 0.01;
    public static double BALANCE_SETPOINT_ANGLE = 0;
    public static double BALANCE_THRESHOLD_DEGREES = 3;
    public static int BALANCE_STEPS_THRESHOLD = 25;

    public static Pose2d[] BLUE_SCORE_POSE = new Pose2d[] {
            new Pose2d(1.425 + trackWidthAdded, 0.453, Rotation2d.fromDegrees(0)), // Score location 1 on blue side
            new Pose2d(1.425 + trackWidthAdded, 1.044, Rotation2d.fromDegrees(0)), // score 2
            new Pose2d(1.425 + trackWidthAdded, 1.579, Rotation2d.fromDegrees(0)), // 3
            new Pose2d(1.425 + trackWidthAdded, 2.204, Rotation2d.fromDegrees(0)), // ...
            new Pose2d(1.425 + trackWidthAdded, 2.773, Rotation2d.fromDegrees(0)),
            new Pose2d(1.425 + trackWidthAdded, 3.273, Rotation2d.fromDegrees(0)),
            new Pose2d(1.425 + trackWidthAdded, 3.831, Rotation2d.fromDegrees(0)),
            new Pose2d(1.425 + trackWidthAdded, 4.433, Rotation2d.fromDegrees(0)),
            new Pose2d(1.425 + trackWidthAdded, 5.082, Rotation2d.fromDegrees(0))
    };

    public static Pose2d[] RED_SCORE_POSE = new Pose2d[] {
            new Pose2d(15.15 - trackWidthAdded, 0.453, Rotation2d.fromDegrees(180)), // Score location 1 on blue side
            new Pose2d(15.15 - trackWidthAdded, 1.044, Rotation2d.fromDegrees(180)), // score 2
            new Pose2d(15.15 - trackWidthAdded, 1.579, Rotation2d.fromDegrees(180)), // 3
            new Pose2d(15.15 - trackWidthAdded, 2.204, Rotation2d.fromDegrees(180)), // ...
            new Pose2d(15.15 - trackWidthAdded, 2.773, Rotation2d.fromDegrees(180)),
            new Pose2d(15.15 - trackWidthAdded, 3.273, Rotation2d.fromDegrees(180)),
            new Pose2d(15.15 - trackWidthAdded, 3.831, Rotation2d.fromDegrees(180)),
            new Pose2d(15.15 - trackWidthAdded, 4.433, Rotation2d.fromDegrees(180)),
            new Pose2d(15.15 - trackWidthAdded, 5.082, Rotation2d.fromDegrees(180))
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

    public static Pose2d[] BLUE_CHARGE_POSE_WAYPOINT =new Pose2d[] {
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

    // bottom to top (farthest from community to closest)
    /* public static Pose2d[] BLUE_START_POSE = new Pose2d[] {
            new Pose2d(2.285, 0.736, Rotation2d.fromDegrees(0)),
            new Pose2d(2.285, 2.638, Rotation2d.fromDegrees(0)),
            new Pose2d(2.285, 4.357, Rotation2d.fromDegrees(0)),
    };

    public static Pose2d[] RED_START_POSE = new Pose2d[] {
            new Pose2d(14.384, 0.736, Rotation2d.fromDegrees(180)),
            new Pose2d(14.384, 2.638, Rotation2d.fromDegrees(180)),
            new Pose2d(14.384, 4.357, Rotation2d.fromDegrees(180)),
    }; */

    public static Pose2d[] BLUE_PARK_POSE = new Pose2d[] {
            new Pose2d(2.324, 4.593, Rotation2d.fromDegrees(0)),
            new Pose2d(4, 0.716, Rotation2d.fromDegrees(0)),
    };

    public static Pose2d[] RED_PARK_POSE = new Pose2d[] {
            new Pose2d(13.847, 4.593, Rotation2d.fromDegrees(180)),
            new Pose2d(12.57, 0.716, Rotation2d.fromDegrees(180)),
    };

    public static double CHARGE_STATION_LOWER_Y = 1.508506;
    public static double CHARGE_STATION_UPPER_Y = 3.978656;

    public static final double BLUE_COMMUNITY_X = 3.34;
    public static final double RED_COMMUNITY_X = 13.2;

    public static double CHARGE_CENTER_Y = 2.75;
}
