package org.firstinspires.ftc.teamcode.teleop;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Intake Sensor Test")
public class ControlAward extends LinearOpMode {


    private Rev2mDistanceSensor distanceSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "intakeSensor");


        waitForStart();


        while (opModeIsActive()){
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if(distance < 7) gamepad1.rumble(100);

            telemetry.addData("Distance (in)", distance);
            telemetry.addData("Detected", distance < 7);
            telemetry.update();
        }
    }
}
