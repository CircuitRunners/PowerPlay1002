package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

public class ParkCommand extends ParallelCommandGroup {



    public ParkCommand(SampleMecanumDrive drive, Lift lift, Arm arm, Claw claw, BeaconDetector.BeaconTags tag, boolean isLeft){

        addCommands(
                new SequentialCommandGroup(
                        new LiftPositionCommand(lift, (int) lift.getLiftPosition() + 40, true),
                        new WaitCommand(500),
                        new RetractOuttakeCommand(lift, arm, claw)
                )

        );

        switch (tag){
            case LEFT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isLeft) ? ThreeCycleTrajectories.leftLeftPark : ThreeCycleTrajectories.rightLeftPark
                        )
                );
                break;
            case CENTER:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isLeft) ? ThreeCycleTrajectories.leftMiddlePark : ThreeCycleTrajectories.rightMiddlePark
                        )
                );
                break;
            case RIGHT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                (isLeft) ? ThreeCycleTrajectories.leftRightPark : ThreeCycleTrajectories.rightRightPark
                        )
                );
                break;
        }

    }
}
