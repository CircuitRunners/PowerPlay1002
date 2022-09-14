package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ChocolateTeleOp extends LinearOpMode {

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

        while(opModeIsActive()){
            if(gamepad1.left_stick_y>=.5){
                leftBack.setPower(gamepad1.left_stick_y*.5);
                rightFront.setPower(gamepad1.left_stick_y*.5);
                leftBack.setPower(gamepad1.left_stick_y*.5);
                rightBack.setPower(gamepad1.left_stick_y*.5);
            }
            if (gamepad1.left_stick_y<=-.5){
                leftBack.setPower(gamepad1.left_stick_y*.5);
                rightFront.setPower(gamepad1.left_stick_y*.5);
                leftBack.setPower(gamepad1.left_stick_y*.5);
                rightBack.setPower(gamepad1.left_stick_y*.5);
            }
        }
    }
}
