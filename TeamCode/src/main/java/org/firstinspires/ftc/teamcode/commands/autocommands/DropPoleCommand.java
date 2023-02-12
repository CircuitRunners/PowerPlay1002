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

public class DropPoleCommand extends ParallelCommandGroup {


    public DropPoleCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft) {

        addCommands(

                new SequentialCommandGroup(
                        new WaitCommand(400),
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH)),
                        new WaitCommand(1050),
                        new InstantCommand(claw::angleUp)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftToPole : ThreeCycleTrajectories.rightToPole
                        ),
//                        new InstantCommand(claw::open),
                        new WaitCommand(100)
                ),
                new SequentialCommandGroup(
//                        new WaitCommand(100),
                        new ParallelRaceGroup(
                                new ProfiledLiftPositionCommand(lift, Lift.LiftPositions.MID.position, true),
                                new WaitCommand(2000)
                        )
//                        new WaitCommand(400),
//                        new InstantCommand(claw::open),
//                        new WaitCommand(300)
                )
        );
    }


}
