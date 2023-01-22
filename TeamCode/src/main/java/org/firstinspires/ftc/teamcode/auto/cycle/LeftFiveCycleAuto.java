package org.firstinspires.ftc.teamcode.auto.cycle;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPoleCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.DropPreloadCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.GoToStackCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.ParkCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.ThreeCycleTrajectories;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BeaconDetector;


@Autonomous(name = "Left 1+5")
public class LeftFiveCycleAuto extends CommandOpMode {


    private SampleMecanumDrive drive;
    private Claw claw;
    private Lift lift;
    private Arm arm;


    private BeaconDetector beaconDetector;
    private BeaconDetector.BeaconTags beaconId = BeaconDetector.BeaconTags.LEFT;

    @Override
    public void initialize() {
//        PhotonCore.enable();
        schedule(new BulkCacheCommand(hardwareMap));

        drive = new SampleMecanumDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        drive.setPoseEstimate(ThreeCycleTrajectories.leftStartingPosition);
        ThreeCycleTrajectories.generateTrajectories(drive);
        claw.close();

        beaconDetector = new BeaconDetector(hardwareMap);

        DropPreloadCommand preloadCommand = new DropPreloadCommand(drive, lift, claw, arm, true);
        GoToStackCommand goToStackCommand1 = new GoToStackCommand(drive, lift, claw, arm, true, 1);
        GoToStackCommand goToStackCommand2 = new GoToStackCommand(drive, lift, claw, arm, true, 2);
        GoToStackCommand goToStackCommand3 = new GoToStackCommand(drive, lift, claw, arm, true, 3);
        GoToStackCommand goToStackCommand4 = new GoToStackCommand(drive, lift, claw, arm, true, 4);
        GoToStackCommand goToStackCommand5 = new GoToStackCommand(drive, lift, claw, arm, true, 5);
        DropPoleCommand dropPoleCommand1 = new DropPoleCommand(drive, lift, claw, arm, true);
        DropPoleCommand dropPoleCommand2 = new DropPoleCommand(drive, lift, claw, arm, true);
        DropPoleCommand dropPoleCommand3 = new DropPoleCommand(drive, lift, claw, arm, true);
        DropPoleCommand dropPoleCommand4 = new DropPoleCommand(drive, lift, claw, arm, true);
        DropPoleCommand dropPoleCommand5 = new DropPoleCommand(drive, lift, claw, arm, true);



        //Start vision
        beaconDetector.startStream();

        while (!isStarted()) {
            beaconId = beaconDetector.update();
            telemetry.addLine("Ready for start!");
            telemetry.addData("Beacon", beaconId);
            telemetry.update();
        }

        beaconDetector.stopStream();
        beaconId = BeaconDetector.BeaconTags.RIGHT;

        schedule(new SequentialCommandGroup(
                preloadCommand,
                goToStackCommand1,
                dropPoleCommand1,
                goToStackCommand2,
                dropPoleCommand2,
                goToStackCommand3,
                dropPoleCommand3,
                goToStackCommand4,
                dropPoleCommand4,
                goToStackCommand5,
                dropPoleCommand5,
                new ParkCommand(drive, lift, arm, claw, beaconId)
        ));
    }

    @Override
    public void run(){
        CommandScheduler.getInstance().run();

        telemetry.addData("lift", lift.getLiftPosition());
        telemetry.update();
    }
}