package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandSnailController extends CommandXboxController {
    public CommandSnailController(int port) {
        super(port);
    }

    public double getDriveForward() {
        if (a().getAsBoolean()) {
            return -applyDeadband(getLeftY());
        } else if (rightBumper().getAsBoolean()) {
            return -applyDeadband(getRightY());
        } else if (leftTrigger().getAsBoolean()) {
            return -applyDeadband(getLeftY());
        }
        return 0;
    }

    public double getDriveTurn() {
        if (a().getAsBoolean()) {
            return applyDeadband(getLeftX());
        } else if (rightBumper().getAsBoolean()) {
            return applyDeadband(getLeftX());
        } else if (leftTrigger().getAsBoolean()) {
            return applyDeadband(getRightX());
        }
        return 0;
    }

    public double getElevatorSpeed() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }

    //these two commands work together to get input from the joystick to control the robot
    //since getLeftBumper is true for driveforward and driveturn, it'll make it so tha tthe
    //left y joystick controls the full movement

    public static double applyDeadband(double value) {
        if (Math.abs(value) < 0.08) return 0;
        else return value;
    }
}