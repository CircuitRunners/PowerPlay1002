package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RetractLiftCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class GoToStackCommand extends ParallelCommandGroup {

    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;

    public GoToStackCommand(SampleMecanumDrive drive, Lift lift, Claw claw, boolean isRed, int cycle){

        this.drive = drive;
        this.lift = lift;
        this.claw = claw;

        int liftPos = 0;
        switch (cycle) {
            case 1:
                liftPos = 1000;
            case 2:
                liftPos = 800;
        }

        addCommands(
                new TrajectorySequenceCommand(
                        drive, isRed ? ThreeCycleTrajectories.redToStack : ThreeCycleTrajectories.blueToStack
                ),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new RetractLiftCommand(lift, claw)
//                        new WaitCommand(300),
//                        new LiftPositionCommand(lift, liftPos, true),
//                        new WaitCommand(500),
//                        new InstantCommand(claw::clampClose)
                )
        );
    }

}
