package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@TeleOp (name = "Skill Issue")
public class SkillIssue extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private double multiplier = 0.5, globalMultiplier = 1;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double y1 = (gamepad1.right_stick_y) * multiplier;
            double x1 = (gamepad1.right_stick_x) * multiplier;

            if (gamepad1.right_bumper) { y1 = 1; x1=0; }

            leftFront.setPower( globalMultiplier * ( y1 - x1 ) );
            leftBack.setPower( globalMultiplier * ( y1 - x1 ));

            rightFront.setPower( globalMultiplier * ( y1 + x1));
            rightBack.setPower( globalMultiplier * ( y1 + x1));
        }

    }
}
