package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
    @AutoLog
    public static class ClawIOInputs {
        public double speed = 0.0;
        public double encoderPosition = 0.0;
        public boolean gamePiece = false;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClawIOInputs inputs) {}

    /** Run the gripper open loop at the specified speed. */
    public default void setSpeed(double speed) {}

    /** Enable or disable brake mode on the gripper. */
    public default void setBrakeMode(boolean enable) {}

    public default void setSensor(boolean enable) {}

    public default boolean getSensor() {return false;}
}
