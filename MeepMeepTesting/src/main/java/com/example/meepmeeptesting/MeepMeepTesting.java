package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        System.setProperty("sun.java2d.opengl", "true");


        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 43, toRadians(230), toRadians(230), 12.1)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(40, -58, toRadians(-90)))
                                        .setReversed(true)
                                        .setAccelConstraint(getAccelerationConstraint(35))
                                        .setTurnConstraint(toRadians(200), toRadians(200))
                                        .splineToConstantHeading(new Vector2d(37, -48.5), toRadians(89))
                                        .splineToConstantHeading(new Vector2d(37, -35), toRadians(90))
                                        .splineTo(new Vector2d(30, -5.0), toRadians(114.8))
                                        .waitSeconds(1) //drop preload
                                        .resetConstraints()

                                        .setReversed(false)
                                        .splineTo(new Vector2d(50, -6.5), toRadians(0)) //to stack
                                        .splineTo(new Vector2d(64.2, -6.5 ), toRadians(0))
                                        .waitSeconds(1) //at stack 1


                                        .setReversed(true)
//                                        .splineTo(new Vector2d(50, -6.5), toRadians(-180))
                                        .splineTo(new Vector2d(32.8, -12), toRadians(-141))
                                        .waitSeconds(1) //at pole 1

                                        .setReversed(false)
                                        .splineTo(new Vector2d(50, -6.0), toRadians(0)) //to stack
                                        .splineTo(new Vector2d(65.0, -6.0), toRadians(0))
                                        .waitSeconds(1) //at stack 2

                                        //repeat cycles

                                        .setReversed(true)
                                        .splineTo(new Vector2d(50, -6.5), toRadians(-180))
                                        .splineTo(new Vector2d(32.8, -12), toRadians(-141))
                                        .waitSeconds(1) //at pole 2

                                        .setReversed(false)
                                        .splineTo(new Vector2d(50, -6.5), toRadians(0)) //to stack
                                        .splineTo(new Vector2d(64.2, -6.5), toRadians(0))
                                        .waitSeconds(1) //at stack 3

                                        .setReversed(true)
                                        .splineTo(new Vector2d(50, -6.5), toRadians(-180))
                                        .splineTo(new Vector2d(32.8, -12), toRadians(-141))
                                        .waitSeconds(1) //at pole 3

                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(15, -7, toRadians(90)), toRadians(180))



//                                        .splineToSplineHeading(new Pose2d(36, -11.7, toRadians(-90)), toRadians(0)) //to stack
//                                        .forward(25)
//                                        .strafeLeft(25)

                                        .build()

                );
//
        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 14)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 43, toRadians(230), toRadians(230), 12.1)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40, -58, toRadians(-90)))
                                 .setReversed(true)
                                        .setAccelConstraint(getAccelerationConstraint(35))
                                        .setTurnConstraint(toRadians(200), toRadians(200))
                                        .splineToConstantHeading(new Vector2d(37, -48.5), toRadians(91))
                                        .splineToConstantHeading(new Vector2d(37, -35), toRadians(90))
                                        .splineTo(new Vector2d(30, -5.0), toRadians(115))
                                        .waitSeconds(1) //drop preload
                                        .resetConstraints()

                                        .setReversed(false)
                                        .splineToSplineHeading(new Pose2d(50, -7.0, toRadians(0)), toRadians(0)) //to stack
//                                        .splineTo(new Vector2d(45.3, -7.0), toRadians(0)) //to stack
                                        .splineToConstantHeading(new Vector2d(64.2, -7.0 ), toRadians(0))
                                        .waitSeconds(1) //at stack 1
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
//                .addEntity(blueBot)
                .start();
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