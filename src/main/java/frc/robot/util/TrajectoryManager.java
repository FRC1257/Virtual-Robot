package frc.robot.util;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.Constants.Drivetrain.*;

public class TrajectoryManager {
    /*
     * PathPlannerTrajectory trajectory = PathPlanner.generatePath(
     * new PathConstraints(4, 4),
     * false,
     * new ArrayList<PathPoint> () {{
     * add(new PathPoint(new Translation2d(), new Rotation2d()));
     * add(new PathPoint(new Translation2d(1, 0), new Rotation2d()));
     * }},
     * new ArrayList<EventMarker> () {{
     * }}
     * 
     * 
     * );
     */

     public enum TrajectoryTypes {
        DriveForward,
        ReverseToPose,
        Straight
     }

     public enum PathRoute {
        ABOVE,
        ACROSS,
        BELOW
     }

     public enum Substation {
        SLIDE,
        TOP,
        BOTTOM
     }

    private DifferentialDriveVoltageConstraint autoVoltageConstraint;

    private TrajectoryConfig config;
    private TrajectoryConfig reverseConfig;

    private Drive drive;

    private boolean flipPose = true;
    private final double speedBoost = 2.5;

    private boolean isBlue = true;

    private PathRoute currentRoute = PathRoute.ABOVE;
    private Substation currentSubstation = Substation.TOP;
    private int scoreNode = 0; // 0 = top, 8 = bottom

    public TrajectoryManager(Drive drive) {
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        ksVolts,
                        kvVoltSecondsPerMeter,
                        kaVoltSecondsSquaredPerMeter),
                drive.getKinematics(),
                10);

        config = new TrajectoryConfig(
                DRIVE_TRAJ_MAX_VEL,
                DRIVE_TRAJ_MAX_ACC)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        reverseConfig = new TrajectoryConfig(
                DRIVE_TRAJ_MAX_VEL,
                DRIVE_TRAJ_MAX_ACC)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        this.drive = drive;
    }

    /*
     * Generate trajectory to drive forward and reverse with config
     */
    public Trajectory forwardTrajectory(Pose2d pose, List<Translation2d> waypoints) {
        if (useSpeedBoost(pose, drive.getPose())) {
            config.setStartVelocity(Math.max(drive.getVelocityMetersPerSecond(), speedBoost));
        } else {
            config.setStartVelocity(drive.getVelocityMetersPerSecond());
        }

        // testing only
        LogInterestingValues(drive.getPose(), FieldConstants.RED_CARGO_POSE[0]);

        // change pose angle to make it faster
        if (flipPose /* && !isReversed(drive.getPose(), pose) */) {
            pose = new Pose2d(pose.getTranslation(), pose.getRotation().minus(Rotation2d.fromDegrees(180)));
        }

        try {
            return TrajectoryGenerator.generateTrajectory(
                drive.getPose(),
                // Pass through these two interior waypoints, making an 's' curve path
                waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                pose,
                config);
        } catch (Exception e) {
            System.out.println("Bad trajectory");
            return null;
        }
    }

    public Trajectory forwardTrajectory(Pose2d pose) {
        return forwardTrajectory(pose, List.of());
    }

    public Trajectory reverseTrajectory(Pose2d pose, List<Translation2d> waypoints) {
        if (useSpeedBoost(pose, drive.getPose())) {
            reverseConfig.setStartVelocity(Math.min(drive.getVelocityMetersPerSecond(), -speedBoost));
        } else {
            reverseConfig.setStartVelocity(drive.getVelocityMetersPerSecond());
        }
        
        // testing only
        LogInterestingValues(drive.getPose(), FieldConstants.BLUE_CHARGE_POSE[1]);
        
        // change pose angle to make it faster
        if (flipPose /* && !isReversed(drive.getPose(), pose) */) {
            pose = new Pose2d(pose.getTranslation(), pose.getRotation().minus(Rotation2d.fromDegrees(180)));
        }

        try {
            return TrajectoryGenerator.generateTrajectory(
                drive.getPose(),
                // Pass through these two interior waypoints, making an 's' curve path
                waypoints,
                // End 3 meters straight ahead of where we started, facing forward
                pose,
                reverseConfig
            );
        } catch (Exception e) {
            System.out.println("Bad trajectory");
            return null;
        }
    }

    public Trajectory reverseTrajectory(Pose2d pose) {
        return reverseTrajectory(pose, List.of());
    }

    // Trajectory going straight to pose
    public Trajectory toPosTrajectory(Pose2d pose) {
        config.setStartVelocity(drive.getVelocityMetersPerSecond());
        LogInterestingValues(drive.getPose(), pose);
        try {

            return TrajectoryGenerator.generateTrajectory(
                drive.getPose(),
                List.of(),
                pose,
                config);
        } catch (Exception e) {
            System.out.println("Bad trajectory");
            return null;
        }
    }

    /*
     * Useful methods for dealing with poses
     */
    public static double angleBetween(Pose2d pose1, Pose2d pose2) {
        return pose2.getRotation().minus(pose1.getRotation()).getDegrees();
    }

    public static boolean isReversed(Pose2d pose1, Pose2d pose2) {
        return Math.abs(angleBetween(pose1, pose2)) > 100;
    }

    public static boolean inFrontOf(Pose2d pose1, Pose2d pose2) {
        return pose2.getTranslation().minus(pose1.getTranslation()).getX() > 0;
    }

    public static double distanceBetween(Pose2d pose1, Pose2d pose2) {
        return pose2.getTranslation().getDistance(pose1.getTranslation());
    }

    public boolean useSpeedBoost(Pose2d pose1, Pose2d pose2) {
        return distanceBetween(pose1, pose2) > 5;
    }

    public void setColor(boolean isBlue) {
        this.isBlue = isBlue;
    }

    public Pose2d getStartPose() {
        return drive.getPose();
    }

    public static Pose2d reversePose2d(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().plus(new Rotation2d(Math.PI)));
    }

    // log things for testing
    // note this is not the proper way to log stuff
    public void LogInterestingValues(Pose2d init, Pose2d end) {
        SmartDashboard.putNumber("Angle between", angleBetween(init, end));
        SmartDashboard.putBoolean("Is reversed", isReversed(init, end));
        SmartDashboard.putBoolean("In front of", inFrontOf(init, end));
    }

    // get trajectory based on type
    public Trajectory getTrajectory(TrajectoryTypes type, Pose2d pose) {
        switch (type) {
            case DriveForward:
                return forwardTrajectory(pose);
            case ReverseToPose:
                return reverseTrajectory(pose);
            default:
                return forwardTrajectory(pose);
        }
    }

    public Trajectory getTrajectory(TrajectoryTypes type, Pose2d pose, List<Translation2d> waypoints) {
        switch (type) {
            case DriveForward:
                return forwardTrajectory(pose, waypoints);
            case ReverseToPose:
                return reverseTrajectory(pose, waypoints);
            default:
                return forwardTrajectory(pose, waypoints);
        }
    }

    public TrajectoryTypes opposite(TrajectoryTypes type) {
        switch (type) {
            case DriveForward:
                return TrajectoryTypes.ReverseToPose;
            case ReverseToPose:
                return TrajectoryTypes.DriveForward;
            default:
                return TrajectoryTypes.DriveForward;
        }
    }

    // get waypoints based on type
    public List<Translation2d> getWaypoints(PathRoute type) {
        // TODO make waypoint addition smarter
        List<Translation2d> waypoints = new ArrayList<>();

        switch (type) {
            case ABOVE:
                Translation2d[] above = getWaypointColorArray()[0];
                waypoints.add(above[0]);
                waypoints.add(above[1]);
                break;
            case ACROSS:
                Translation2d[] mid = getWaypointColorArray()[1];
                waypoints.add(mid[0]);
                waypoints.add(mid[1]);
                break;
            case BELOW:
            default:
                Translation2d[] below = getWaypointColorArray()[2];
                waypoints.add(below[0]);
                waypoints.add(below[1]);
                break;
        }

        if (isBlue) {
            waypoints.add(FieldConstants.BLUE_SUBSTATION_ENTRY);
        } else {
            waypoints.add(FieldConstants.RED_SUBSTATION_ENTRY);
        }

        return waypoints;
    }

    public CommandBase backUpAndTurnWaypoint(Pose2d waypoint) {
        // TODO only backup and turn when neccecary
        Pose2d backupPose = new Pose2d(waypoint.getTranslation().plus(new Translation2d(0, isBlue ? 0.5 : -0.5)), waypoint.getRotation());
        double angle = backupPose.getRotation().minus(drive.getPose().getRotation()).getDegrees();

        return getOnTheFlyCommand(TrajectoryTypes.ReverseToPose, backupPose).andThen(drive.turnExactAngle(angle));
    }

    private Pose2d getSubstationPose() {
        // TODO add stuff here
        if (isBlue) {
            return new Pose2d(FieldConstants.BLUE_SUBSTATION_ENTRY, Rotation2d.fromDegrees(0));
        } else {
            return new Pose2d(FieldConstants.RED_SUBSTATION_ENTRY, Rotation2d.fromDegrees(180));
        }
    }

    public CommandBase goToSubstation(PathRoute type) {
        List<Translation2d> waypoints = getWaypoints(type);
        Pose2d pose = getSubstationPose();

        return backUpAndTurnWaypoint(new Pose2d(waypoints.get(waypoints.size()-1), Rotation2d.fromDegrees(isBlue ? 0 : 180))).andThen(getOnTheFlyCommand(TrajectoryTypes.DriveForward, pose, waypoints));
    }

    public Translation2d[][] getWaypointColorArray() {
        if (isBlue) {
            return FieldConstants.BLUE_WAYPOINTS;
        } else {
            return FieldConstants.RED_WAYPOINTS;
        }
    }

    public CommandBase getOnTheFlyCommand(TrajectoryTypes type, Pose2d pose) {
        return new CommandBase() {
            @Override
            public void initialize() {
                // TODO use a smarter way to get the trajectory then try everything
                // I tried this and it worked but its not computationally fast
                // who cares
                Trajectory trajectory = getTrajectory(type, pose);
                if (trajectory == null) {
                    trajectory = getTrajectory(opposite(type), pose);
                }
                if (trajectory == null) {
                    flipPose = !flipPose;
                    trajectory = getTrajectory(type, pose);
                    if (trajectory == null) {
                        trajectory = getTrajectory(opposite(type), pose);
                    }
                }
                drive.driveTrajectory(trajectory);
            }
            
            @Override
            public void execute() {
                drive.followTrajectory();
            }

            @Override
            public void end(boolean interrupted) {
                drive.stop();
            }

            @Override
            public boolean isFinished() {
                return drive.isTrajectoryFinished();
            }
        };
    }

    public CommandBase getOnTheFlyCommand(TrajectoryTypes type, Pose2d pose, List<Translation2d> waypoints) {
        return new CommandBase() {
            @Override
            public void initialize() {
                // TODO use a smarter way to get the trajectory then try everything
                // I tried this and it worked but its not computationally fast
                // who cares
                Trajectory trajectory = getTrajectory(type, pose, waypoints);
                if (trajectory == null) {
                    trajectory = getTrajectory(opposite(type), pose, waypoints);
                }
                if (trajectory == null) {
                    flipPose = !flipPose;
                    trajectory = getTrajectory(type, pose, waypoints);
                    if (trajectory == null) {
                        trajectory = getTrajectory(opposite(type), pose, waypoints);
                    }
                }
                drive.driveTrajectory(trajectory);
            }
            
            @Override
            public void execute() {
                drive.followTrajectory();
            }

            @Override
            public void end(boolean interrupted) {
                drive.stop();
            }

            @Override
            public boolean isFinished() {
                return drive.isTrajectoryFinished();
            }
        };
    }

    // schedule command
    public void RunOnTheFly(TrajectoryTypes type, Pose2d pose) {
        CommandScheduler.getInstance().schedule(getOnTheFlyCommand(type, pose));
    }

    public void scheduleSmartMotionCommand(Pose2d pose) {
        // check if angle is too great and rotation required
        if (Math.abs(angleBetween(drive.getPose(), pose)) > 90) {
            // CommandScheduler.getInstance().schedule(turnIfNecessary(pose));
            System.out.println("Turning");
        }

        // check if its easier to reverse or go forward
        CommandBase driveCommand;
        if (isReversed(drive.getPose(), pose)) {
            driveCommand = getOnTheFlyCommand(TrajectoryTypes.DriveForward, pose);
        } else {
            driveCommand = getOnTheFlyCommand(TrajectoryTypes.ReverseToPose, pose);
        }

        // Turn to proper pose
        CommandBase turnCommand = drive.turnExactAngle(pose.getRotation().getDegrees());

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(driveCommand, turnCommand));
    }

    public CommandBase turnIfNecessary(Pose2d pose) {
        // check if need to rotate
        if (Math.abs(angleBetween(drive.getPose(), pose)) < 90 || drive.getVelocityMetersPerSecond() > 1) {
            return new InstantCommand();
        }

        return drive.turnAngleCommand(angleBetween(drive.getPose(), pose) + 180);
    }

    public CommandBase goToPos(Pose2d pose) {
        // check if need to rotate
        return new SequentialCommandGroup(
            turnIfNecessary(pose),
            getOnTheFlyCommand(TrajectoryTypes.DriveForward, pose),
            drive.turnExactAngle(pose.getRotation().getDegrees())
        );
    }

    public CommandBase goToPosBackwards(Pose2d pose) {
        // check if need to rotate
        return new SequentialCommandGroup(
            getOnTheFlyCommand(TrajectoryTypes.ReverseToPose, pose),
            drive.turnExactAngle(pose.getRotation().getDegrees())
        );
    }

    public void scheduleDriveForward(Pose2d pose) {
        CommandScheduler.getInstance().schedule(goToPos(pose));
    }
}
