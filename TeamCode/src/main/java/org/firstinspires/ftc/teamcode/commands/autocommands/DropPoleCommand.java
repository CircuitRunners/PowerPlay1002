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

public class DropPoleCommand extends ParallelCommandGroup {


    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;
    private Arm arm;

    public DropPoleCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft) {
        this.drive = drive;
        this.lift = lift;
        this.claw = claw;
        this.arm = arm;

        addCommands(

                new SequentialCommandGroup(
                        new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH)),
                        new WaitCommand(1100),
                        new InstantCommand(claw::angleUp)
                ),
                new TrajectorySequenceCommand(
                        drive, isLeft ? ThreeCycleTrajectories.leftToPole : ThreeCycleTrajectories.rightToPole
                ),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new LiftPositionCommand(lift, Lift.LiftPositions.HIGH.position, true),
                        new WaitCommand(1300),
                        new InstantCommand(claw::open)
                )
        );
    }


}
