package org.firstinspires.ftc.teamcode.auto.park;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;


@Autonomous (name="Parking Auto")
public class ParkAuto extends CommandOpMode {


    private SampleMecanumDrive drive;
    private Claw claw;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    private Pose2d startPose = new Pose2d(0, 0, toRadians(0.0));

    @Override
    public void initialize(){
        schedule(new BulkCacheCommand(hardwareMap));

        claw = new Claw(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        beaconDetector = new BeaconDetector(hardwareMap);

        //Start vision
        beaconDetector.startStream();

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

        //Close the claw
        claw.clampClose();

        while(opModeInInit()){
            beaconId = beaconDetector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

        beaconDetector.stopStream();

        //Gets much more complex, for now simply switch
        switch(beaconId){
            case LEFT:
                schedule(new TrajectorySequenceCommand(drive, leftTrajectoryAbs));
                break;
            case CENTER:
                schedule(new TrajectorySequenceCommand(drive, middleTrajectoryAbs));
                break;
            case RIGHT:
                schedule(new TrajectorySequenceCommand(drive, rightTrajectoryAbs));
                break;
        }


    }



}
