package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;

@Autonomous
public class BlueParkAuto extends CommandOpMode {


    private SampleMecanumDrive drive;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        beaconDetector = new BeaconDetector(hardwareMap);

        //Start vision
        beaconDetector.startStream();
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(40)
                .build();

        while(!isStarted()){
            beaconId = beaconDetector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

        beaconDetector.stopStream();

        //Actually do stuff
        schedule(
                new TrajectoryCommand(drive, traj1)
        );

    }



}