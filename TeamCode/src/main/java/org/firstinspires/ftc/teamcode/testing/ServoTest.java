package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ServoTest extends LinearOpMode {

    private Servo clawServo;

    @Override
    public void runOpMode(){

        clawServo = hardwareMap.get(Servo.class, "servoImport");

        waitForStart();
        boolean rightBumperPrev = false;
        boolean clawClosed = false;
        while(opModeIsActive()){
            if(gamepad1.right_bumper && !rightBumperPrev){
                if(clawClosed) clawServo.setPosition(0);
                else clawServo.setPosition(1);
                clawClosed = !clawClosed;
            }
            rightBumperPrev = gamepad1.right_bumper;
        }



    }
}
