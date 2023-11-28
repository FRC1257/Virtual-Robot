package frc.robot.subsystems.pivotWrist;

import org.littletonrobotics.junction.AutoLog;

public interface PivotWristIO {
    @AutoLog
    public static class PivotWristIOInputs {
        public double angle = 0.0;
        public double angleRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public double PIVOT_WRIST_MAX_ANGLE = 150.0;
    public double PIVOT_WRIST_MIN_ANGLE = -30.0;

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotWristIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double motorVolts) {
    }

    /** Returns the current distance measurement. */
    public default double getAngle() {
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
