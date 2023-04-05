package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.stream.Collectors;

@TeleOp
public class ServoScanner extends LinearOpMode {

    private String deviceName = "poleGuide";

    ServoImplEx thisCrazyServo;
    ArrayList<Double> storedValues = new ArrayList<Double>(2);

    @Override
    public void runOpMode() throws InterruptedException {
        thisCrazyServo = hardwareMap.get(ServoImplEx.class, deviceName);
        thisCrazyServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        if (gamepad1.left_bumper) {
            storedValues.add((double) gamepad1.left_stick_y);
        }

        waitForStart();

        while (opModeIsActive()) {
            thisCrazyServo.setPosition(gamepad1.left_stick_y);
            telemetry.addData("Position", thisCrazyServo.getPosition());
            telemetry.addData("Values", storedValues.stream().map(Object::toString)
                    .collect(Collectors.joining(", ")));
            telemetry.update();
        }
    }

}
