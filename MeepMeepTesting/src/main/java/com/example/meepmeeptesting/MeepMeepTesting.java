package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        System.setProperty("sun.java2d.opengl", "true");


        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, toRadians(220), toRadians(220), 11.78)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(37, -58, toRadians(-90)))
                                        .setReversed(true)
//                                        .splineTo(new Vector2d(36, -25), toRadians(91))
//                                        .splineTo(new Vector2d(36, -25), toRadians(93))
                                        .splineToLinearHeading(new Pose2d(30, -4.5, toRadians(-49)), toRadians(141))
                                        .waitSeconds(1) //drop preload

                                        .setReversed(false)
                                        .splineTo(new Vector2d(40, -4), toRadians(-2)) //to stack
                                        .splineTo(new Vector2d(60, -4), toRadians(0))
                                        .waitSeconds(1) //at stack 1


                                        .setReversed(true)
                                        .splineTo(new Vector2d(40, -5.5), toRadians(-180))
                                        .splineTo(new Vector2d(27, -5.5), toRadians(-132))
                                        .waitSeconds(1) //at pole 1

                                        .setReversed(false)
                                        .splineTo(new Vector2d(40, -11.7), toRadians(0)) //to stack
                                        .splineTo(new Vector2d(60, -11.7), toRadians(0))
                                        .waitSeconds(1) //at stack 2

                                        //repeat cycles

                                        .setReversed(true)
                                        .splineTo(new Vector2d(40, -11.7), toRadians(-180))
                                        .splineTo(new Vector2d(31, -15), toRadians(-132))
                                        .waitSeconds(1) //at pole 2

                                        .setReversed(false)
                                        .splineTo(new Vector2d(40, -11.7), toRadians(0)) //to stack
                                        .splineTo(new Vector2d(60, -11.7), toRadians(0))
                                        .waitSeconds(1) //at stack 3

                                        .setReversed(true)
                                        .splineTo(new Vector2d(40, -11.7), toRadians(-180))
                                        .splineTo(new Vector2d(31, -15), toRadians(-132))
                                        .waitSeconds(1) //at pole 3


//                                        .splineToSplineHeading(new Pose2d(36, -11.7, toRadians(-90)), toRadians(0)) //to stack
//                                        .forward(25)
//                                        .strafeLeft(25)

                                        .build()

                );
//
//        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
//                .setDimensions(14, 14)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(40, 40, toRadians(220), toRadians(220), 11.78)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-37, 63, toRadians(-90)))
//                                .splineTo(new Vector2d(-36, 30), toRadians(-90))
//                                .splineTo(new Vector2d(-36, 18), toRadians(-86))
//                                .splineToSplineHeading(new Pose2d(-29.5, 16.5, toRadians(50)), toRadians(45))
//                                .build()
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
//                .addEntity(blueBot)
                .start();
    }
}