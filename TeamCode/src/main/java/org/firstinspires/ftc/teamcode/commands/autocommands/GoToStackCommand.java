package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.commands.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class GoToStackCommand extends ParallelCommandGroup {


    public GoToStackCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft, int cycle){


        int liftPos = 0;
        switch (cycle) {
            case 1:
                liftPos = 300;
                break;
            case 2:
                liftPos = 250;
                break;
            case 3:
                liftPos = 200;
                break;
        }

        addCommands(
                new TrajectorySequenceCommand(
                        drive, isLeft ? ThreeCycleTrajectories.leftToStack : ThreeCycleTrajectories.rightToStack
                ),
                new InstantCommand(claw::angleDown),
                new InstantCommand(claw::open),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.DOWN))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new ProfiledLiftPositionCommand(lift, liftPos, true)
                )
        );
    }

}
