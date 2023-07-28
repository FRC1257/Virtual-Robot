package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double motorVolts) {
    }

    /** Returns the current distance measurement. */
    public default double getDistance() {
        return 0.0;
    }

    public default void setPIDConstants(double p, double i, double d, double ff) {
    }

    /** Go to Setpoint */
    public default void goToSetpoint(double setpoint) {
    }

}
