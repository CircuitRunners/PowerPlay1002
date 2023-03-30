package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Tank Drive")
public class Test extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;


    @Override
    public void runOpMode(){
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){


            leftFront.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            leftBack.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            rightFront.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
            rightBack.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }

    }
}
