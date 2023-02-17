package org.firstinspires.ftc.teamcode.commands.autocommands;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
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


    public static final Pose2d rightStartingPosition = new Pose2d(43, -56.8, toRadians(-90));
    public static final Pose2d leftStartingPosition = new Pose2d(-40, -56.8, toRadians(-90));

    public static final Pose2d rightPreloadPosition =
            new Pose2d(29.0, -6.0, toRadians(115));

    public static final Pose2d leftPreloadPosition =
            new Pose2d(-29.0, -4.0, toRadians(-119.2));

    public static final Pose2d rightDropPosition =
            new Pose2d(36.2, -13.5, toRadians(36.2));

    public static final Pose2d leftDropPosition =
            new Pose2d(-33.1, -11.8, toRadians(143.8)); //-10.4

    public static final Pose2d rightStackPosition =
            new Pose2d(64.1, -5.5, toRadians(0));

    public static final Pose2d leftStackPosition =
            new Pose2d(-64.1, -6.5, toRadians(180));

    public static TrajectorySequence rightPreloadToPole;
    public static TrajectorySequence leftPreloadToPole;

    public static TrajectorySequence rightToStackPreload;
    public static TrajectorySequence leftToStackPreload;
    public static TrajectorySequence rightToStack;
    public static TrajectorySequence leftToStack;
    public static TrajectorySequence rightToStackDrifted;
    public static TrajectorySequence leftToStackDrifted;

    public static TrajectorySequence rightToPole;
    public static TrajectorySequence leftToPole;

    public static TrajectorySequence leftLeftPark;
    public static TrajectorySequence leftMiddlePark;
    public static TrajectorySequence leftRightPark;
    public static TrajectorySequence rightLeftPark;
    public static TrajectorySequence rightMiddlePark;
    public static TrajectorySequence rightRightPark;

    public static MarkerCallback clawCloseAction = () -> {};
    public static MarkerCallback clawOpenAction = () -> {};
    private static final double endCloseActionScale = 0.961;
    private static final double endOpenActionScale = 0.952;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        rightPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.rightStartingPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(33))
                        .setVelConstraint(getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(37, -48.5), toRadians(89))
                        .splineToConstantHeading(new Vector2d(37, -35), toRadians(90))
                        .splineTo(rightPreloadPosition.vec(), toRadians(118.3))
                        .resetConstraints()
                        .build();

        leftPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.leftStartingPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(30))
                        .setVelConstraint(getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(-37, -48.5), toRadians(91))
                        .splineToConstantHeading(new Vector2d(-37, -35), toRadians(90))
                        .splineTo(leftPreloadPosition.vec(), toRadians(65.2))
                        .resetConstraints()
                        .build();
        //###################################################################

        rightToPole =
                drive.trajectorySequenceBuilder(rightStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(50, -5.5), toRadians(180))
                        .splineTo(rightDropPosition.vec(), toRadians(-144.0))
                        .build();

        leftToPole =
                drive.trajectorySequenceBuilder(leftStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, -6.5), toRadians(0))
                        .splineTo(leftDropPosition.vec(), toRadians(-35.9))
                        .build();

        //###########################################################################

        rightToStackPreload =
                drive.trajectorySequenceBuilder(rightPreloadToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(50, -5.5), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStackPreload =
                drive.trajectorySequenceBuilder(leftPreloadToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(-50, -6.5), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightToStack =
                drive.trajectorySequenceBuilder(rightToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(50, -5.5), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftToStack =
                drive.trajectorySequenceBuilder(leftToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(-50, -6.5), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightToStackDrifted =
                drive.trajectorySequenceBuilder(rightToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(50, -4.7), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec().plus(new Vector2d(0, 0.7)), toRadians(0))
                        .build();

        leftToStackDrifted =
                drive.trajectorySequenceBuilder(leftToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(-50, -5.8), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec().plus(new Vector2d(0, 0.7)), toRadians(180))
                        .build();



        //###########################################################################

        //PARKING
        leftLeftPark =
                drive.trajectorySequenceBuilder(leftToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(-62, -7), toRadians(180))
                        .build();

        leftMiddlePark =
                drive.trajectorySequenceBuilder(leftToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(90)))
                        .build();

        leftRightPark =
                drive.trajectorySequenceBuilder(leftToPole.end())
                        .setConstraints(getVelocityConstraint(48, toRadians(270), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(48))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-16, -6, toRadians(90)))
//                        .forward(6)
//                        .turn(toRadians(-43))
//                        .strafeRight(25)
                        .build();

        rightLeftPark =
                drive.trajectorySequenceBuilder(rightToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(62, -5.7), toRadians(0))
                        .build();

        rightMiddlePark =
                drive.trajectorySequenceBuilder(rightToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(88)))
                        .lineToLinearHeading(new Pose2d(37, -5.7, toRadians(90)))
                        .build();

        rightRightPark =
                drive.trajectorySequenceBuilder(rightToPole.end())
                        .setConstraints(getVelocityConstraint(48, toRadians(270), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(48))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(27, -5.5, toRadians(90)))
//                        .forward(6)
//                        .turn(toRadians(-43))
//                        .strafeRight(25)
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
