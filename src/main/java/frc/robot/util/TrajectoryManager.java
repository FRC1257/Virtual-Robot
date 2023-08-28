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
        GoToPos,
        GoToPos2,
        Straight

     }

    private DifferentialDriveVoltageConstraint autoVoltageConstraint;

    private TrajectoryConfig config;
    private TrajectoryConfig reverseConfig;

        private Drive drive;

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

    public Trajectory exampleTrajectory() {
        // testing only
        config.setStartVelocity(drive.getVelocityMetersPerSecond());
        LogInterestingValues(drive.getPose(), FieldConstants.RED_CARGO_POSE[0]);
        try {
            return TrajectoryGenerator.generateTrajectory(
                drive.getPose(),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(/* new Translation2d(1, 1), new Translation2d(2, -1) */),
                // End 3 meters straight ahead of where we started, facing forward
                FieldConstants.RED_CHARGE_POSE[0],
                config);
        } catch (Exception e) {
            System.out.println("Bad trajectory");
            return null;
        }
    }

    public Trajectory exampleTrajectory2() {
        // testing only
        config.setStartVelocity(drive.getVelocityMetersPerSecond());
        LogInterestingValues(drive.getPose(), FieldConstants.BLUE_CHARGE_POSE[1]);
        try {
            return TrajectoryGenerator.generateTrajectory(
                drive.getPose(),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(/* new Translation2d(1, 1), new Translation2d(2, -1) */),
                // End 3 meters straight ahead of where we started, facing forward
                FieldConstants.BLUE_CHARGE_POSE[1],
                config);
        } catch (Exception e) {
            System.out.println("Bad trajectory");
            return null;
        }
    }

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

    public static double angleBetween(Pose2d pose1, Pose2d pose2) {
        return pose1.getRotation().minus(pose2.getRotation()).getDegrees();
    }

    public static boolean isReversed(Pose2d pose1, Pose2d pose2) {
        return Math.abs(angleBetween(pose1, pose2)) > 100;
    }

    public static boolean inFrontOf(Pose2d pose1, Pose2d pose2) {
        return pose2.getTranslation().minus(pose1.getTranslation()).getX() > 0;
    }

    public void LogInterestingValues(Pose2d init, Pose2d end) {
        SmartDashboard.putNumber("Angle between", angleBetween(init, end));
        SmartDashboard.putBoolean("Is reversed", isReversed(init, end));
        SmartDashboard.putBoolean("In front of", inFrontOf(init, end));
    }

    public static Pose2d reversePose2d(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().plus(new Rotation2d(Math.PI)));
    }

    public Trajectory getTrajectory(TrajectoryTypes type) {
        switch (type) {
            case GoToPos:
                return exampleTrajectory();
            case GoToPos2:
                return exampleTrajectory2();
            default:
                return exampleTrajectory();
        }
    }

    public CommandBase getOnTheFlyCommand(TrajectoryTypes type) {
        return new CommandBase() {
            @Override
            public void initialize() {
                Trajectory trajectory = getTrajectory(type);
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

    public void RunOnTheFly(TrajectoryTypes type) {
        CommandScheduler.getInstance().schedule(getOnTheFlyCommand(type));
    }

    public CommandBase turnIfNecessary(Pose2d pose) {
        // check if need to rotate

        return drive.turnAngleCommand(angleBetween(pose, drive.getPose()) + 180);
    }

    public CommandBase goToPos(Pose2d pose) {
        // check if need to rotate
        return new SequentialCommandGroup(
            turnIfNecessary(pose),
            getOnTheFlyCommand(TrajectoryTypes.GoToPos),
            drive.turnExactAngle(pose.getRotation().getDegrees())
        );
    }

    public void scheduleGoToPos(Pose2d pose) {
        CommandScheduler.getInstance().schedule(goToPos(pose));
    }
}
