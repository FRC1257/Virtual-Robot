package frc.robot.subsystems.laterator;

import org.littletonrobotics.junction.AutoLog;

public interface LateratorIO {
    @AutoLog
    public static class LateratorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMeters = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public double LATERATOR_MAX_HEIGHT = 36.0;
    public double LATERATOR_MIN_HEIGHT = 0.0;

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LateratorIOInputs inputs) {
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

    public default void setBrake(boolean brake) {
    }

    public default boolean atSetpoint() {
        return false;
    }

    public default void setP(double p) {}
    
    public default void setI(double i) {}

    public default void setD(double d) {}

    public default void setFF(double ff) {}

    public default double getP() { return 0.0; }

    public default double getI() { return 0.0; }

    public default double getD() { return 0.0; }

    public default double getFF() { return 0.0; }

}
