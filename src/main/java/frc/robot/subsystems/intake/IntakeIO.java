package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double angle = 0.0;
        public double angleRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double motorVolts) {
    }

    public default void setBrake(boolean brake) {
    }
}
