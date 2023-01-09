package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class ThreeCycleTrajectories {


    public static final Pose2d rightStartingPosition = new Pose2d(37, -58, toRadians(-90));
    public static final Pose2d leftStartingPosition = new Pose2d(-37, -56.8, toRadians(-90));

    public static final Pose2d rightPreloadPosition =
            new Pose2d(29, -4.5, toRadians(135));

    public static final Pose2d leftPreloadPosition =
            new Pose2d(-30.5, -4.5, toRadians(-119));

    //TODO: Change to be the left side auto, not the blue
    public static final Pose2d rightDropPosition =
            new Pose2d(31, -15, toRadians(-132));

    public static final Pose2d leftDropPosition =
            new Pose2d(-34, -7.5, toRadians(-51));

    public static final Pose2d rightStackPosition =
            new Pose2d(60, -11.7, toRadians(0));

    public static final Pose2d leftStackPosition =
            new Pose2d(-68, -4, toRadians(180));

    public static TrajectorySequence rightPreloadToPole;
    public static TrajectorySequence leftPreloadToPole;

    public static TrajectorySequence rightToStack;
    public static TrajectorySequence leftToStack;

    public static TrajectorySequence rightToStackPreload;
    public static TrajectorySequence leftToStackPreload;

    public static TrajectorySequence rightToPole;
    public static TrajectorySequence leftToPole;

    public static TrajectorySequence leftLeftPark;
    public static TrajectorySequence leftMiddlePark;
    public static TrajectorySequence leftRightPark;
//    public static TrajectorySequence rightLeftPark;
//    public static TrajectorySequence rightMiddlePark;
//    public static TrajectorySequence rightRightPark;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        rightPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.rightStartingPosition)
                        .setReversed(true)
//                        .splineTo(new Vector2d(36, -25), toRadians(91))
                        .splineTo(new Vector2d(36, -16), toRadians(93))
                        .splineTo(rightPreloadPosition.vec(), toRadians(141))
                        .build();

        //TODO: this
        leftPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.leftStartingPosition)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, -25), toRadians(89))
                        .splineTo(new Vector2d(-36, -25), toRadians(87))
                        .splineToSplineHeading(leftPreloadPosition, toRadians(42))
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
                        .splineTo(new Vector2d(-40, -4), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightToStackPreload =
                drive.trajectorySequenceBuilder(rightPreloadPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(40, -8), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStackPreload =
                drive.trajectorySequenceBuilder(leftPreloadPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(-40, -7), toRadians(-130)) //to stack
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
                        .splineTo(new Vector2d(-40, -7.5), toRadians(0))
                        .splineTo(leftDropPosition.vec(), toRadians(-52))
                        .build();

        leftLeftPark =
                drive.trajectorySequenceBuilder(new Pose2d(leftDropPosition.vec(), toRadians(130)))
                        .forward(6)
                        .turn(toRadians(45))
                        .strafeLeft(25)
                        .build();

        leftMiddlePark =
                drive.trajectorySequenceBuilder(new Pose2d(leftDropPosition.vec(), toRadians(130)))
                        .forward(6)
                        .turn(toRadians(45))
                        .build();

        leftLeftPark =
                drive.trajectorySequenceBuilder(new Pose2d(leftDropPosition.vec(), toRadians(130)))
                        .forward(6)
                        .turn(toRadians(45))
                        .strafeRight(25)
                        .build();
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}
