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


public class DropPreloadCommand extends ParallelCommandGroup {


    public DropPreloadCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft) {

        addCommands(

                new SequentialCommandGroup(
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH)),
                        new WaitCommand(1500),
                        new InstantCommand(claw::angleUp)
                ),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new TrajectorySequenceCommand(
                                drive, isLeft ? ThreeCycleTrajectories.leftPreloadToPole : ThreeCycleTrajectories.rightPreloadToPole
                        )
                ),
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new LiftPositionCommand(lift, Lift.LiftPositions.HIGH.position, true),
                        new WaitCommand(1700),
                        new InstantCommand(claw::open)
                )
        );
    }


}
