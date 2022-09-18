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
    private double multiplier = 0.5;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double y1 = (gamepad1.right_stick_y) * multiplier;
            double x1 = (gamepad1.right_stick_x) * multiplier;

            if (gamepad1.right_bumper) { y1 = 1; x1=0; }

            leftFront.setPower( ( y1 - x1 ) );
            leftBack.setPower(  ( y1 - x1 ));

            rightFront.setPower(( y1 + x1));
            rightBack.setPower(  ( y1 + x1));
            if ((gamepad1.right_trigger > 0.5) && gamepad1.y) {
                multiplier = 1;
            }   else if ((gamepad1.right_trigger > 0.5) && gamepad1.b){
                multiplier = 0.5;
            }

        }

    }
}
