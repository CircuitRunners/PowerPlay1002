package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class GoToStackCommand extends ParallelCommandGroup {


    public GoToStackCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft, int cycle) {


        // Have the liftPos set to something relevant in case something goes wonky
        int liftPos = 161;

        switch (cycle) {
            case 1:
                liftPos = 161;
                break;
            case 2:
                liftPos = 112;
                break;
            case 3:
                liftPos = 76;
                break;
            // Theoretical Values based on Pattern
            case 4:
                liftPos = 144; // **-6
                break;
            case 5:
                liftPos = 138; // **-6
                break;
        }

        addCommands(
                new SequentialCommandGroup(
                        (cycle != 1) ? new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToStack : ThreeCycleTrajectories.rightToStack
                        ) : new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToStackPreload : ThreeCycleTrajectories.rightToStackPreload
                        ),
                        new InstantCommand(claw::close),
                        new WaitCommand(400)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(claw::angleDown),
                        new WaitCommand(1100), ////changed from 1200
                        new InstantCommand(claw::fullOpen) //Wait for the arm to get all the way down before fully opening the claw
                ),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.DOWN))
                ),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new LiftPositionCommand(lift, liftPos, true)
                )
        );
    }

}
