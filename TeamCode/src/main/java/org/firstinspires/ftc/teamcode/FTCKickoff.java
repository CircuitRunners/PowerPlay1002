package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class FTCKickoff extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class,"leftMotor");
        waitForStart();
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0.5);
        rightMotor.setPower(-.5);

    }
}
