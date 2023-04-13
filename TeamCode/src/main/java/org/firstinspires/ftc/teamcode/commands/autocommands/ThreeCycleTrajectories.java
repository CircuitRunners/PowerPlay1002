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
            new Pose2d(30.0, -6.0, toRadians(115));

    public static final Pose2d leftPreloadPosition =
            new Pose2d(-29.0, -4.0, toRadians(-119.2));

    public static final Pose2d rightMediumPosition =
            new Pose2d(35.0, -13.5, toRadians(36.2));

    public static final Pose2d leftMediumPosition =
            new Pose2d(-33.1, -11.8, toRadians(143.8)); //-10.4

    public static final Pose2d rightHighPosition =
            new Pose2d(35.0, -2.0, toRadians(115));

    public static final Pose2d leftHighPosition =
            new Pose2d(-38.0, -1.0, toRadians(-119.2));

    public static final Pose2d rightStackPosition =
            new Pose2d(65.0, -5.5, toRadians(0));

    public static final Pose2d leftStackPosition =
            new Pose2d(-65, -6.5, toRadians(180)); //red left field 1: -64.5, blue left field 2 : -64.6


    //Preload
    public static TrajectorySequence rightPreloadToPole;
    public static TrajectorySequence leftPreloadToPole;

    //Stack
    public static TrajectorySequence rightToStackPreload;
    public static TrajectorySequence leftToStackPreload;
    public static TrajectorySequence rightToStack;
    public static TrajectorySequence leftToStack;
    public static TrajectorySequence rightToStackDrifted;
    public static TrajectorySequence leftToStackDrifted;

    public static TrajectorySequence rightHighToStack;
    public static TrajectorySequence leftHighToStack;
    public static TrajectorySequence rightHighToStackDrifted;
    public static TrajectorySequence leftHighToStackDrifted;

    //Pole
    public static TrajectorySequence rightToPole;
    public static TrajectorySequence leftToPole;

    public static TrajectorySequence rightHighToPole;
    public static TrajectorySequence leftHighToPole;

    //Parking

    public static TrajectorySequence leftLeftPark;
    public static TrajectorySequence leftMiddlePark;
    public static TrajectorySequence leftRightPark;
    public static TrajectorySequence rightLeftPark;
    public static TrajectorySequence rightMiddlePark;
    public static TrajectorySequence rightRightPark;

    public static TrajectorySequence leftHighLeftPark;
    public static TrajectorySequence leftHighMiddlePark;
    public static TrajectorySequence leftHighRightPark;
    public static TrajectorySequence rightHighLeftPark;
    public static TrajectorySequence rightHighMiddlePark;
    public static TrajectorySequence rightHighRightPark;

    public static MarkerCallback clawCloseAction = () -> {};
    public static MarkerCallback clawOpenAction = () -> {};
    private static final double endCloseActionScale = 0.965;
    private static final double endOpenActionScale = 0.952;


    public static void generateTrajectories(SampleMecanumDrive drive) {

        rightPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.rightStartingPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(31))
                        .setVelConstraint(getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(37, -48.5), toRadians(89))
                        .splineToConstantHeading(new Vector2d(37, -35), toRadians(90))
                        .splineTo(rightPreloadPosition.vec(), toRadians(117.0))
                        .resetConstraints()
                        .build();

        leftPreloadToPole =
                drive.trajectorySequenceBuilder(ThreeCycleTrajectories.leftStartingPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .setAccelConstraint(getAccelerationConstraint(31))
                        .setVelConstraint(getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setTurnConstraint(toRadians(200), toRadians(200))
                        .splineToConstantHeading(new Vector2d(-37, -48.5), toRadians(91))
                        .splineToConstantHeading(new Vector2d(-37, -35), toRadians(90))
                        .splineTo(leftPreloadPosition.vec(), toRadians(67.4))
                        .resetConstraints()
                        .build();

        //###########################################################################
        //###################################################################

        rightToPole =
                drive.trajectorySequenceBuilder(rightStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(50, -5.5), toRadians(180))
                        .splineTo(rightMediumPosition.vec(), toRadians(-144.0))
                        .build();

        leftToPole =
                drive.trajectorySequenceBuilder(leftStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(-50, -6.5), toRadians(0))
                        .splineTo(leftMediumPosition.vec(), toRadians(-35.9))
                        .build();

        //TODO: high

        rightHighToPole =
                drive.trajectorySequenceBuilder(rightStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(50, -5.5), toRadians(180))
                        .splineTo(rightHighPosition.vec(), toRadians(144.0))
                        .build();

        leftHighToPole =
                drive.trajectorySequenceBuilder(leftStackPosition)
                        .addTemporalMarker(endOpenActionScale, 0.0, clawOpenAction)
                        .setReversed(true)
                        .splineTo(new Vector2d(-55, -5.5), toRadians(0))
                        .splineTo(leftHighPosition.vec(), toRadians(39))
                        .build();

        //###########################################################################
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

        //############################################################

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


        //TODO: high
        rightHighToStack =
                drive.trajectorySequenceBuilder(rightHighToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(60, -5.5), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec(), toRadians(0))
                        .build();

        leftHighToStack =
                drive.trajectorySequenceBuilder(leftHighToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(-60, -5.5), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec(), toRadians(180))
                        .build();

        rightHighToStackDrifted =
                drive.trajectorySequenceBuilder(rightHighToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(60, -4.7), toRadians(0)) //to stack
                        .splineTo(rightStackPosition.vec().plus(new Vector2d(0, 0.7)), toRadians(0))
                        .build();

        leftHighToStackDrifted =
                drive.trajectorySequenceBuilder(leftHighToPole.end())
                        .addTemporalMarker(endCloseActionScale, 0.0, clawCloseAction)
                        .setReversed(false)
                        .splineTo(new Vector2d(-60, -4.8), toRadians(180)) //to stack
                        .splineTo(leftStackPosition.vec().plus(new Vector2d(0, 0.7)), toRadians(180))
                        .build();



        //###########################################################################
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
                        .lineToLinearHeading(new Pose2d(-14, -5, toRadians(90)))
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
                        .lineToLinearHeading(new Pose2d(26.5, -5.5, toRadians(90)))
                        .build();

        //TODO: high
        leftHighLeftPark =
                drive.trajectorySequenceBuilder(leftHighToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(-62, -7), toRadians(180))
                        .build();

        leftHighMiddlePark =
                drive.trajectorySequenceBuilder(leftHighToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(-90)))
                        .build();

        leftHighRightPark =
                drive.trajectorySequenceBuilder(leftHighToPole.end())
                        .setConstraints(getVelocityConstraint(48, toRadians(270), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(48))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-14, -5, toRadians(-90)))
                        .build();

        rightHighLeftPark =
                drive.trajectorySequenceBuilder(rightHighToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))
                        .setReversed(false)
                        .splineTo(new Vector2d(62, -5.7), toRadians(0))
                        .build();

        rightHighMiddlePark =
                drive.trajectorySequenceBuilder(rightHighToPole.end())
                        .setConstraints(getVelocityConstraint(47, toRadians(260), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(47))                        .lineToLinearHeading(new Pose2d(-37, -7, toRadians(88)))
                        .lineToLinearHeading(new Pose2d(37, -5.7, toRadians(-90)))
                        .build();

        rightHighRightPark =
                drive.trajectorySequenceBuilder(rightHighToPole.end())
                        .setConstraints(getVelocityConstraint(48, toRadians(270), DriveConstants.TRACK_WIDTH), getAccelerationConstraint(48))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(26.5, -5.5, toRadians(-90)))
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
