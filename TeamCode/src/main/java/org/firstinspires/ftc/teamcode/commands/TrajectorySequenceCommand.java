package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private TrajectorySequence trajectory;

    public TrajectorySequenceCommand(SampleMecanumDrive drive, TrajectorySequence trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

//    @Override
//    public void initialize(){
//        // Doing nothing here rn
//    }

    @Override
    public void execute(){
        drive.followTrajectorySequence(trajectory);
    }

    /*
    public void end(boolean inturrupted) {
        do Things when done
    }
     */

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }


}
