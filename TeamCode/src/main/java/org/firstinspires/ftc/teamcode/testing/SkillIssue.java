package org.firstinspires.ftc.teamcode.testing;

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

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        double lock = 0.0;
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y1 = (gamepad1.right_stick_y);
            if (lock != 0) y1 = lock;
            if (gamepad1.right_bumper) {
                lock = y1;
            } else if (gamepad1.left_bumper) {
                lock = 0;
            }
            double x1 = (gamepad1.right_stick_x);
            double y2 = -(gamepad1.left_stick_y);
            double x2 = (gamepad1.left_stick_x);

            double denominator = Math.max(Math.abs(y1) + Math.abs(x1), 1);

            leftFront.setPower( ( y1 - x1 )/denominator );
            leftBack.setPower(  ( y1 - x1 )/denominator);

            rightFront.setPower(( y1 + x1)/denominator);
            rightBack.setPower(  ( y1 + x1)/denominator);
        }

    }
}
