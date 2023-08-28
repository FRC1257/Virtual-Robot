package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TrajectoryManager;

public class GenerateOnTheFly extends CommandBase {

    private Drive drive;
    private Trajectory trajectory;
    private TrajectoryManager.TrajectoryTypes trajectoryType;
    private TrajectoryManager trajectoryManager;

    /**
     * 
     */
    public GenerateOnTheFly(Drive drive, TrajectoryManager.TrajectoryTypes trajectoryType, TrajectoryManager trajectoryManager) {
        this.drive = drive;
        this.trajectoryType = trajectoryType;
        this.trajectoryManager = trajectoryManager;

        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        drive.stop();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        drive.followTrajectory();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        trajectory = trajectoryManager.getTrajectory(trajectoryType);
        drive.driveTrajectory(trajectory);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return drive.isTrajectoryFinished();
    }
    
}
