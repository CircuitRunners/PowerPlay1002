package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftPositionCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class DropPoleCommand extends ParallelCommandGroup {


    public DropPoleCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft) {

        addCommands(

                new SequentialCommandGroup(
                        new WaitCommand(600),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH)),
                        new WaitCommand(1300),
                        new InstantCommand(claw::angleUp)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToPole : ThreeCycleTrajectories.rightToPole
                        )
                ),
                new SequentialCommandGroup(
//                        new WaitCommand(100),
                        new ProfiledLiftPositionCommand(lift, Lift.LiftPositions.MID.position, true),
                        new WaitCommand(700),
                        new InstantCommand(claw::open),
                        new WaitCommand(300)
                )
        );
    }


}
