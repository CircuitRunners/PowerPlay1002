package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp (name = "Servo Wiggler")
public class Tester extends LinearOpMode {


    private Servo claw;
    private Servo leftArm;
    private Servo rightArm;


    @Override
    public void runOpMode() {
        claw = hardwareMap.get(ServoImplEx.class, "clawServo");
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) claw.setPosition(0);
            else claw.setPosition(1);

            if (gamepad1.b) {
                leftArm.setPosition(0);
                rightArm.setPosition(0);
            } else {
                leftArm.setPosition(0.3);
                rightArm.setPosition(0.3);
            }
        }
    }
}
