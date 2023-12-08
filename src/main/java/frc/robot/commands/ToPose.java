package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TrajectoryManager;

public class ToPose extends Command {
    
    private Drive drive;
    private Pose2d pose;
    private TrajectoryManager manager;
    private Command command;

    public ToPose(Drive drive, TrajectoryManager manager, Pose2d pose) {
        this.drive = drive;
        this.pose = pose;
        this.manager = manager;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.command = this.manager.goToPos(pose);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

}
