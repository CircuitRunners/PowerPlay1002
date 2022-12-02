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

                new LiftPositionCommand(lift, 1600, true),
                new SequentialCommandGroup(
                        new TrajectorySequenceCommand(
                                drive, isRed ? ThreeCycleTrajectories.redPreloadToPole : ThreeCycleTrajectories.bluePreloadToPole
                        ),
                        new WaitCommand(690),
                        new InstantCommand(claw::clampOpen)
                )
        );
    }


}
