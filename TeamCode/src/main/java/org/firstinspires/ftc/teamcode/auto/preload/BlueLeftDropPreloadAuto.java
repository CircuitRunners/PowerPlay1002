package org.firstinspires.ftc.teamcode.auto.preload;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RetractLiftCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

@Autonomous(name = "Blue Left Side Pre-Load")
public class BlueLeftDropPreloadAuto extends CommandOpMode {


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
                .forward(5)
                .turn(toRadians(90))
                .forward(30)
                .turn(toRadians(-90))
                .forward(26)
                .turn(toRadians(45))
                .build();

        Trajectory forwardToPole = drive.trajectoryBuilder(driveToPole.end())
                .forward(7)
                .build();

        Trajectory backFromPole = drive.trajectoryBuilder(forwardToPole.end())
                .back(7)
                .build();


        TrajectorySequence leftTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .turn(toRadians(-45))
                .build();
        TrajectorySequence middleTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .turn(toRadians(-135))
                .forward(25)
                .turn(toRadians(90))
                .build();
        TrajectorySequence rightTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .turn(toRadians(-135))
                .forward(50)
                .turn(toRadians(90))
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
                new LiftPositionCommand(lift, 200),
                new WaitCommand(300),
                new TrajectorySequenceCommand(drive, driveToPole),
                new WaitCommand(1000),
                new LiftPositionCommand(lift, 2800),
                new InstantCommand(() -> lift.setLiftPower(0.1)),
                new TrajectoryCommand(drive, forwardToPole),
                new WaitCommand(2000),
                new InstantCommand(claw::clampOpen),
                new WaitCommand(500),
                new InstantCommand(claw::clampClose),
                new TrajectoryCommand(drive, backFromPole),
                new WaitCommand(500),
                new RetractLiftCommand(lift, claw),
                new WaitCommand(500),
                new SelectCommand(() -> {
                    switch (beaconId) {
                        case LEFT:
                            return new TrajectorySequenceCommand(drive, leftTrajectoryAbs);
                        case CENTER:
                            return new TrajectorySequenceCommand(drive, middleTrajectoryAbs);
                        default:
                            return new TrajectorySequenceCommand(drive, rightTrajectoryAbs);
                    }
                })
        ));
        

    }


}
