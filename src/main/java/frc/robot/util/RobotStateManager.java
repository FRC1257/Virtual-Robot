package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotStateManager {
    public enum States {
        TRAVEL_TO_PICKUP,
        PICKUP,
        TRAVEL_TO_GRID,
        SCORE
    }

    private States currentState = States.TRAVEL_TO_PICKUP;

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

    public boolean checkTravelToPickup() {
        return currentState == States.TRAVEL_TO_PICKUP;
    }
    
}
