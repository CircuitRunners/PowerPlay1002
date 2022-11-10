package org.firstinspires.ftc.teamcode.auto.twocycle;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RetractLiftCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

@Autonomous(name = "Red Right Side Two Cycle")
public class RedRightDropTwoAuto extends CommandOpMode {


    private SampleMecanumDrive drive;
    private Claw claw;
    private Lift lift;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));

    @Override
    public void initialize() {
        schedule(new BulkCacheCommand(hardwareMap));

        drive = new SampleMecanumDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        drive.setPoseEstimate(startPose);
        claw.clampClose();

        beaconDetector = new BeaconDetector(hardwareMap);


        TrajectorySequence driveToPole = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(6)
                .forward(28)
//                .back(6)
                .turn(toRadians(46))
                .build();

        TrajectorySequence forwardToPole = drive.trajectorySequenceBuilder(driveToPole.end())
                .forward(9)
                .build();

        TrajectorySequence backFromPole = drive.trajectorySequenceBuilder(forwardToPole.end())
                .back(9)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(backFromPole.end())
                .turn(toRadians(-45))
                .forward(24)
//                .back(6.5)
                .turn(toRadians(-90))
                .build();

        TrajectorySequence forwardToStack = drive.trajectorySequenceBuilder(goToStack.end())
                .forward(24.1)
                .build();

        TrajectorySequence backFromStack = drive.trajectorySequenceBuilder((forwardToStack.end()))
                .back(24)
                .turn(toRadians(-135))
                .build();

        TrajectorySequence secondForwardToPole = drive.trajectorySequenceBuilder(backFromStack.end())
                .forward(9)
                .build();

        TrajectorySequence secondBackFromPole = drive.trajectorySequenceBuilder(secondForwardToPole.end())
                .back(8)
                .turn(toRadians(-45))
                .forward(24)
                .build();

        TrajectorySequence leftTrajectoryAbs = drive.trajectorySequenceBuilder(secondBackFromPole.end())
                .strafeRight(25)
                .build();

        TrajectorySequence middleTrajectoryAbs = drive.trajectorySequenceBuilder(secondBackFromPole.end())
                .forward(1)
                .build();

        TrajectorySequence rightTrajectoryAbs = drive.trajectorySequenceBuilder(secondBackFromPole.end())
                .strafeLeft(25)
                .build();


        //Start vision
        beaconDetector.startStream();

        while (!isStarted()) {
            beaconId = beaconDetector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

        beaconDetector.stopStream();

        schedule(new SequentialCommandGroup(
                new LiftPositionCommand(lift, 600),
                new WaitCommand(200),
                new TrajectorySequenceCommand(drive, driveToPole),
                new WaitCommand(300),
                new LiftPositionCommand(lift, 2000),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new TrajectorySequenceCommand(drive, forwardToPole),
                new WaitCommand(500),
                new LiftPositionCommand(lift,1900),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new WaitCommand(300),
                new InstantCommand(claw::clampOpen),
                new WaitCommand(500),
                new InstantCommand(claw::clampClose),
                new TrajectorySequenceCommand(drive, backFromPole),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new RetractLiftCommand(lift, claw),
                        new TrajectorySequenceCommand(drive, goToStack)
                ),
                new LiftPositionCommand(lift, 1000),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new TrajectorySequenceCommand(drive, forwardToStack),
                new LiftPositionCommand(lift, 420),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new WaitCommand(200),
                new InstantCommand(claw::clampClose),
                new WaitCommand(400),
                new LiftPositionCommand(lift, 1000),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new ParallelCommandGroup(
                        new TrajectorySequenceCommand(drive, backFromStack),
                        new SequentialCommandGroup(
                                new LiftPositionCommand(lift,2000),
                                new InstantCommand(() -> lift.setLiftPower(0.1))
                        )
                ),
                new TrajectorySequenceCommand(drive, secondForwardToPole),
                new LiftPositionCommand(lift,1900),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new WaitCommand(500),
                new InstantCommand(claw::clampOpen),
                new WaitCommand(500),
                new InstantCommand(claw::clampClose),
                new ParallelCommandGroup(
                        new TrajectorySequenceCommand(drive, secondBackFromPole),
                        new RetractLiftCommand(lift, claw)
                ),
                new SelectCommand(() -> {
                    switch (beaconId) {
                        case LEFT:
                            return new TrajectorySequenceCommand(drive, leftTrajectoryAbs);
                        case RIGHT:
                            return new TrajectorySequenceCommand(drive, rightTrajectoryAbs);
                        default:
                            return new TrajectorySequenceCommand(drive, middleTrajectoryAbs);
                    }
                })
        ));

    }


}
