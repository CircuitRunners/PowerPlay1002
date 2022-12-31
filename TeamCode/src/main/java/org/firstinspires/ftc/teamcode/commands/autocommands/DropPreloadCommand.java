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


public class DropPreloadCommand extends ParallelCommandGroup {

    // make trajectories static so .end

    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;
    private Arm arm;

    public DropPreloadCommand(SampleMecanumDrive drive, Lift lift, Claw claw, Arm arm, boolean isLeft) {
        this.drive = drive;
        this.lift = lift;
        this.claw = claw;
        this.arm = arm;


        addCommands(

                new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH)),
                new TrajectorySequenceCommand(
                        drive, isLeft ? ThreeCycleTrajectories.leftPreloadToPole : ThreeCycleTrajectories.rightPreloadToPole
                ),
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        new LiftPositionCommand(lift, Lift.LiftPositions.HIGH.position, true),
                        new InstantCommand(claw::angleUp),
                        new WaitCommand(1000),
                        new InstantCommand(claw::open)
                )
        );
    }


}
