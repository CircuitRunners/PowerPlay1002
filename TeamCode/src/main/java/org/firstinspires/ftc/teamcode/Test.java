package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Test extends LinearOpMode {

    private DcMotor leftFront;

    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotor.class, "lf");

        waitForStart();
        leftFront.setPower(0.5);


    }
}
