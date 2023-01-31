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


    public static final Pose2d rightStartingPosition = new Pose2d(40, -56.8, toRadians(-90));
    public static final Pose2d leftStartingPosition = new Pose2d(-40, -56.8, toRadians(-90));

    public static final Pose2d rightPreloadPosition =
            new Pose2d(30, -5.0, toRadians(135));

    public static final Pose2d leftPreloadPosition =
            new Pose2d(-30, -5.0, toRadians(-119.2));

    public static final Pose2d rightDropPosition =
            new Pose2d(32.8, -10.4, toRadians(-132));

    public static final Pose2d leftDropPosition =
            new Pose2d(-32.8, -10.4, toRadians(129));

    public static final Pose2d rightStackPosition =
            new Pose2d(64.4, -7, toRadians(0));

    public static final Pose2d leftStackPosition =
            new Pose2d(-64.4, -7, toRadians(180));

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
    public static TrajectorySequence rightLeftPark;
    public static TrajectorySequence rightMiddlePark;
    public static TrajectorySequence rightRightPark;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        rightPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.rightStartingPosition)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(30))
                        .setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(37, -48.5), toRadians(89))
                        .splineToConstantHeading(new Vector2d(37, -35), toRadians(90))
                        .splineTo(rightPreloadPosition.vec(), toRadians(114.8))
                        .resetConstraints()
                        .build();

        leftPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.leftStartingPosition)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(30))
                        .setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(-37, -48.5), toRadians(91))
                        .splineToConstantHeading(new Vector2d(-37, -35), toRadians(90))
                        .splineTo(leftPreloadPosition.vec(), toRadians(65.2))
                        .resetConstraints()
                        .build();

        rightToStackPreload =
                drive.trajectorySequenceBuilder(rightPreloadPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(45.3, -7), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStackPreload =
                drive.trajectorySequenceBuilder(leftPreloadPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(-45.3, -7), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightToPole =
                drive.trajectorySequenceBuilder(rightStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(45.3, -7), toRadians(180))
                        .splineTo(rightDropPosition.vec(), toRadians(-136))
                        .build();

        leftToPole =
                drive.trajectorySequenceBuilder(leftStackPosition)
                        .setReversed(true)
                        .splineTo(new Vector2d(-45.3, -7), toRadians(0))
                        .splineTo(leftDropPosition.vec(), toRadians(-44))
                        .build();

        rightToStack =
                drive.trajectorySequenceBuilder(rightDropPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(45.3, -7), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStack =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setReversed(false)
                        .splineTo(new Vector2d(-45.3, -7), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        //PARKING
        leftLeftPark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(-62, -7), toRadians(180))
//                        .forward(6)
//                        .turn(toRadians(-43))
//                        .strafeLeft(23)
                        .build();

        leftMiddlePark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(88)))
                        .build();

        leftRightPark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
//                        .splineToLinearHeading(new Pose2d(-18, -5, toRadians(90)), toRadians(0))
                        .forward(6)
                        .turn(toRadians(-43))
                        .strafeRight(25)
                        .build();

        rightLeftPark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(62, -7), toRadians(0))
                        .build();

        rightMiddlePark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(88)))
                        .lineToLinearHeading(new Pose2d(37, -7, toRadians(88)))
                        .build();

        rightRightPark =
                drive.trajectorySequenceBuilder(leftDropPosition)
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        //.splineToLinearHeading(new Pose2d(-18, -5, toRadians(90)), toRadians(0))
                        .forward(6)
                        .turn(toRadians(-43))
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
