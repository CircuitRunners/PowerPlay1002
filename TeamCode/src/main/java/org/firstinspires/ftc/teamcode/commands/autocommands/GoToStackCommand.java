package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class GoToStackCommand extends ParallelCommandGroup {


    public GoToStackCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft, int cycle) {


        // Have the liftPos set to something relevant in case something goes wonky
        int liftPos = 161;

        switch (cycle) {
            case 1:
                liftPos = 160;
                break;
            case 2:
                liftPos = 121;
                break;
            case 3:
                liftPos = 84;
                break;
            case 4:
                liftPos = 42;
                break;
            case 5:
                liftPos = 2;
                break;
        }
        addCommands(
                new SequentialCommandGroup(
                        (cycle != 1) ?
                                (cycle >= 4) ? new TrajectorySequenceCommand(
                                        drive, isLeft ? ThreeCycleTrajectories.leftToStackDrifted : ThreeCycleTrajectories.rightToStackDrifted
                                ) :
                                        new TrajectorySequenceCommand(
                                                drive, isLeft ? ThreeCycleTrajectories.leftToStack : ThreeCycleTrajectories.rightToStack
                                        )
                                : new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToStackPreload : ThreeCycleTrajectories.rightToStackPreload
                        ),
                        new WaitCommand(200)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new InstantCommand(claw::angleDown),
                        new WaitCommand(800),
                        new InstantCommand(claw::fullOpen) //Wait for the arm to get all the way down before fully opening the claw
//                        new WaitCommand(600),
//                        new InstantCommand(claw::close),
//                        new WaitCommand(200)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.DOWN))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new ParallelRaceGroup(
                                new ProfiledLiftPositionCommand(lift, liftPos, true),
                                new WaitCommand(1900)
                        )
                )
        );
    }

}
