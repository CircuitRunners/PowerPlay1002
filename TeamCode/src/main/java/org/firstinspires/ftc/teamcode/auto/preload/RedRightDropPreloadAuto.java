package org.firstinspires.ftc.teamcode.auto.preload;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

@Autonomous (name="Red Right Side Pre-Load")
public class RedRightDropPreloadAuto extends CommandOpMode {


    private SampleMecanumDrive drive;
    private Claw claw;
    private Lift lift;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));

    @Override
    public void initialize(){
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


        TrajectorySequence leftTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(toRadians(90))
                .forward(25)
                .turn(toRadians(-90))
                .build();
        TrajectorySequence middleTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .build();
        TrajectorySequence rightTrajectoryAbs = drive.trajectorySequenceBuilder(startPose)
                .forward(25)
                .turn(toRadians(-90))
                .forward(25)
                .turn(toRadians(90))
                .build();


        //Start vision
        beaconDetector.startStream();

        while(!isStarted()){
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
                new InstantCommand(claw::clampOpen)
        ));

//        //Gets much more complex, for now simply switch
//        switch(beaconId){
//            case LEFT:
//                schedule(new TrajectorySequenceCommand(drive, leftTrajectoryAbs));
//                break;
//            case CENTER:
//                schedule(new TrajectorySequenceCommand(drive, middleTrajectoryAbs));
//                break;
//            case RIGHT:
//                schedule(new TrajectorySequenceCommand(drive, rightTrajectoryAbs));
//                break;
//        }


    }



}
