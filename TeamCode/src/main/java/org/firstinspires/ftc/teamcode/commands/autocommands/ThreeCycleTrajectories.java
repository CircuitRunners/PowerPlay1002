package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ThreeCycleTrajectories {


    public static final Pose2d rightStartingPosition = new Pose2d(37, -58, toRadians(-90));
    public static final Pose2d leftStartingPosition = new Pose2d(-37, -58, toRadians(-90));

    public static final Pose2d rightPreloadPosition =
            new Pose2d(31, -7, toRadians(135));

    public static final Pose2d leftPreloadPosition =
            new Pose2d(-29, -5.5, toRadians(45));

    //TODO: Change to be the left side auto, not the blue
    public static final Pose2d rightDropPosition =
            new Pose2d(31, -15, toRadians(-132));

    public static final Pose2d leftDropPosition =
            new Pose2d(-31, -15, toRadians(-48));

    public static final Pose2d rightStackPosition =
            new Pose2d(60, -11.7, toRadians(0));

    public static final Pose2d leftStackPosition =
            new Pose2d(-60, -11.7, toRadians(180));

    public static TrajectorySequence rightPreloadToPole;
    public static TrajectorySequence leftPreloadToPole;

    public static TrajectorySequence rightToStack;
    public static TrajectorySequence leftToStack;

    public static TrajectorySequence rightToPole;
    public static TrajectorySequence leftToPole;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        rightPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.rightStartingPosition)
                        .setReversed(true)
//                        .splineTo(new Vector2d(36, -25), toRadians(91))
                        .splineTo(new Vector2d(36, -16), toRadians(91))
                        .splineTo(rightPreloadPosition.vec(), toRadians(131))
                        .build();

        //TODO: this
        leftPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.leftStartingPosition)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, -25), toRadians(89))
                        .splineTo(new Vector2d(-36, -16), toRadians(89))
                        .splineTo(leftPreloadPosition.vec(), toRadians(49.5))
                        .build();

        rightToStack =
                drive.trajectorySequenceBuilder(rightDropPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(40, -11.7), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStack =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(-40, -11.7), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightToPole =
                drive.trajectorySequenceBuilder(rightStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(40, -11.7), toRadians(180))
                        .splineTo(rightDropPosition.vec(), toRadians(-132))
                        .build();

        leftToPole =
                drive.trajectorySequenceBuilder(leftStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, -11.7), toRadians(0))
                        .splineTo(leftDropPosition.vec(), toRadians(-48))
                        .build();
    }

}
