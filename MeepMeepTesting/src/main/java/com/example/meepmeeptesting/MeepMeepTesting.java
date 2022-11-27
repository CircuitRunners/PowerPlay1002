package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        System.setProperty("sun.java2d.opengl", "true");

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14, 14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, toRadians(220), toRadians(220), 11.78)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37, -63, toRadians(90)))
                                .splineTo(new Vector2d(36, -40), toRadians(94))
                                .splineTo(new Vector2d(30, -30), toRadians(135))
                                .waitSeconds(2) //drop preload
                                .back(10)
                                .lineToLinearHeading(new Pose2d(37, -11.7, toRadians(0))) //to stack
                                .lineTo(new Vector2d(60, -11.7))
                                .waitSeconds(1) //at stack
                                .setReversed(true) //to pole
                                .splineTo(new Vector2d(40, -11.7), toRadians(-180))
                                .splineToSplineHeading(new Pose2d(29.5, -16.5, toRadians(-130)), toRadians(-135))
                                .waitSeconds(1) //at pole
//                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(40, -11.7, toRadians(0)), toRadians(0)) //to stack
                                .splineTo(new Vector2d(60, -11.7), toRadians(0))
                                .waitSeconds(1) //at stack

                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}