package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class ThreeCycleTrajectories {


    public static final Pose2d redStartingPosition = new Pose2d(37, -58, toRadians(90));
    public static final Pose2d blueStartingPosition = new Pose2d(-37, 63, toRadians(-90));

    public static final Pose2d redDropPosition =
            new Pose2d(30, -15, toRadians(-130));

    public static final Pose2d blueDropPosition =
            new Pose2d(-30, 15, toRadians(50));

    public static final Pose2d redStackPosition =
            new Pose2d(-60, 11.7, toRadians(0));

    public static final Pose2d blueStackPosition =
            new Pose2d(60, -11.7, toRadians(180));

    public static TrajectorySequence redPreloadToPole;
    public static TrajectorySequence bluePreloadToPole;

    public static TrajectorySequence redToStack;
    public static TrajectorySequence blueToStack;

    public static TrajectorySequence redToPole;
    public static TrajectorySequence blueToPole;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        redPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.redStartingPosition)
                        .splineTo(new Vector2d(36, -30), toRadians(90))
                        .splineTo(new Vector2d(36, -22), toRadians(94))
                        .splineToSplineHeading(redDropPosition, toRadians(-137))
                        .build();

        bluePreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.blueStartingPosition)
                        .splineTo(new Vector2d(-36, 30), toRadians(-90))
                        .splineTo(new Vector2d(-36, 22), toRadians(-86))
                        .splineToSplineHeading(blueDropPosition, toRadians(42))
                        .build();

        redToStack =
                drive.trajectorySequenceBuilder(redDropPosition)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(40, -11.7, toRadians(0)), toRadians(0)) //to stack
                        .splineTo(redStackPosition.vec(), toRadians(0))
                        .build();

        blueToStack =
                drive.trajectorySequenceBuilder(blueDropPosition)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-40, 11.7, toRadians(180)), toRadians(180)) //to stack
                        .splineTo(blueStackPosition.vec(), toRadians(180))
                        .build();

        redToPole =
                drive.trajectorySequenceBuilder(redStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(40, -11.7), toRadians(-180))
                        .splineToSplineHeading(redDropPosition, toRadians(-135))
                        .build();

        blueToPole =
                drive.trajectorySequenceBuilder(blueStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, 11.7), toRadians(0))
                        .splineToSplineHeading(blueDropPosition, toRadians(45))
                        .build();
    }

}
