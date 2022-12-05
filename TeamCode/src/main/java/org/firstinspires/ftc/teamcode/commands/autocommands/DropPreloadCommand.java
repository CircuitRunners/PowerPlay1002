package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class DropPreloadCommand extends ParallelCommandGroup {

    // make trajectories static so .end

    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;

    public DropPreloadCommand(SampleMecanumDrive drive, Lift lift, Claw claw, boolean isRed) {
        this.drive = drive;
        this.lift = lift;
        this.claw = claw;


        addCommands(

                new LiftPositionCommand(lift, 600, true),
                new TrajectorySequenceCommand(
                        drive, isRed ? ThreeCycleTrajectories.redPreloadToPole : ThreeCycleTrajectories.bluePreloadToPole
                )
//                new SequentialCommandGroup(
//                        new WaitCommand(1500),
//                        new LiftPositionCommand(lift, 2000, true),
//                        new WaitCommand(1000),
//                        new LiftPositionCommand(lift, 1900, true),
//                        new InstantCommand(claw::clampOpen)
//                )
        );
    }


}
