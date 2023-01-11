package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@TeleOp
public class DriveMotorFrictionFF extends LinearOpMode {


    DcMotorEx left;
    DcMotorEx right;


    private boolean prevUp = false;
    private boolean prevSet = false;


    ArrayList<DcMotorEx> motors = new ArrayList<>();

    /*03
    0
    LF: 0.
    LB: 0.03265
    RF: 0.0434
    RB: 0.0425
     */
    @Override
    public void runOpMode() {


        left = hardwareMap.get(DcMotorEx.class, "leftLift");
        right = hardwareMap.get(DcMotorEx.class, "rightLift");


        motors.add(left);
        motors.add(right);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT));


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            telemetry.addData("Current left", left.getPower());
            telemetry.addData("Current right", right.getPower());


            if (gamepad1.dpad_up && !prevUp) {
                left.setPower(left.getPower() + 0.005);
                right.setPower(right.getPower() - 0.005);
                prevSet = true;
            }

            prevUp = gamepad1.dpad_up;

            if (gamepad1.left_bumper) {
                left.setPower(0);
                right.setPower(0);
            }


            telemetry.update();
        }
    }
}

