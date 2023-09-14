package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class RobotStateManager {
    public enum States {
        TRAVEL_TO_PICKUP,
        PICKUP,
        TRAVEL_TO_GRID,
        SCORE
    }

    private States currentState = States.TRAVEL_TO_PICKUP;
    private int[] node = new int[] {-1, -1}; // group and node
    private int substation = -1; // 0 = bottom, 1 = top, 2 = slide
    private static RobotStateManager instance;

    public static RobotStateManager getInstance() {
        if (instance == null) {
            instance = new RobotStateManager();
        }
        return instance;
    }

    public RobotStateManager() {}

    public void reset() {
        currentState = States.TRAVEL_TO_PICKUP;
    }

    public States getState() {
        return currentState;
    }

    public void advanceState() {
        switch (currentState) {
            case TRAVEL_TO_PICKUP:
                currentState = States.PICKUP;
                break;
            case PICKUP:
                currentState = States.TRAVEL_TO_GRID;
                break;
            case TRAVEL_TO_GRID:
                currentState = States.SCORE;
                break;
            case SCORE:
                currentState = States.TRAVEL_TO_PICKUP;
                break;
        }
    }
    
    public void updateStateFromPosition(Pose2d pose, boolean isBlue) {
        if (isBlue) {
            if (pose.getX() < FieldConstants.feet(16)) {
                currentState = States.TRAVEL_TO_GRID;
            } else if (pose.getX() > FieldConstants.feet(32)) {
                currentState = States.TRAVEL_TO_PICKUP;
            } else {
                currentState = States.PICKUP;
            }
        } else {
            if (pose.getX() > FieldConstants.feet(37)) {
                currentState = States.TRAVEL_TO_GRID;
            } else if (pose.getX() < FieldConstants.feet(22)) {
                currentState = States.TRAVEL_TO_PICKUP;
            } else {
                currentState = States.PICKUP;
            }
        }
    }

    public void setGroup(int group) {
        node[0] = group;
        substation = group;

        SmartDashboard.putNumber("Substation", substation);
        SmartDashboard.putNumber("Node Group", node[0]);
    }

    public void setNode(int node) {
        this.node[1] = node;
        SmartDashboard.putNumber("Node", this.node[1]);
    }

    private Pose2d getPoseNode(boolean isBlue) {
        int node_num = node[0] * 3 + node[1];
        if (node_num <= -1) {
            node_num = 0;
        }

        if (isBlue) {
            return FieldConstants.BLUE_SCORE_POSE[node_num];
        } else {
            return FieldConstants.RED_SCORE_POSE[node_num];
        }
    }

    private Pose2d getPoseSubstation(boolean isBlue) {
        int substationLocal = this.substation;
        if (substationLocal <= -1) {
            substationLocal = 0;
        }

        if (isBlue) {
            return FieldConstants.BLUE_PICKUP[substationLocal];
        } else {
            return FieldConstants.RED_PICKUP[substationLocal];
        }
    }

    private Pose2d getPoseFromState(boolean isBlue) {
        switch (currentState) {
            case TRAVEL_TO_PICKUP:
                return getPoseSubstation(isBlue);
            case PICKUP:
                return getPoseSubstation(isBlue);
            case TRAVEL_TO_GRID:
                return getPoseNode(isBlue);
            case SCORE:
                return getPoseNode(isBlue);
            default:
                return getPoseSubstation(isBlue);
        }
    }

    public void scheduleMovement(TrajectoryManager manager) {
        updateStateFromPosition(manager.getStartPose(), DriverStation.getAlliance() == DriverStation.Alliance.Blue);
        if (currentState == States.PICKUP || currentState == States.SCORE) {
            return;
        }
        CommandScheduler.getInstance().schedule(
            manager.goToPos(getPoseFromState(DriverStation.getAlliance() == DriverStation.Alliance.Blue))
        );
        resetNode();
    }

    public CommandBase getMovement(TrajectoryManager manager) {
        updateStateFromPosition(manager.getStartPose(), DriverStation.getAlliance() == DriverStation.Alliance.Blue);
        if (currentState == States.PICKUP || currentState == States.SCORE) {
            return new InstantCommand();
        }
        return manager.goToPos(getPoseFromState(DriverStation.getAlliance() == DriverStation.Alliance.Blue)).andThen(new InstantCommand(() -> resetNode()));
    }

    public void toggleInputs(int group) {
        if (substation == -1) {
            setGroup(group);
            return;
        }

        if (node[1] == -1) {
            setNode(group);
            return;
        }

        resetNode();
    }

    private void resetNode() {
        node = new int[] {-1, -1};
        substation = -1;

        SmartDashboard.putNumber("Substation", substation);
        SmartDashboard.putNumber("Node Group", node[0]);
        SmartDashboard.putNumber("Node", node[1]);
    }
    
}
