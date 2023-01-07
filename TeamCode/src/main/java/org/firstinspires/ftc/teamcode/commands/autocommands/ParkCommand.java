package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

public class ParkCommand extends ParallelCommandGroup {



    public ParkCommand(SampleMecanumDrive drive, Lift lift, Arm arm, Claw claw, BeaconDetector.BeaconTags tag){

        addCommands(new RetractOuttakeCommand(lift, arm, claw));

        switch (tag){
            case LEFT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                ThreeCycleTrajectories.leftLeftPark
                        )
                );
                break;
            case CENTER:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                ThreeCycleTrajectories.leftMiddlePark
                        )
                );
                break;
            case RIGHT:
                addCommands(
                        new TrajectorySequenceCommand(drive,
                                ThreeCycleTrajectories.leftRightPark
                        )
                );
                break;
        }

    }
}
