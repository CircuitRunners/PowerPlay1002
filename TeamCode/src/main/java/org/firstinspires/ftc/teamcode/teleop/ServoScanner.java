package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class ServoScanner extends LinearOpMode {

    ServoImplEx thisCrazyServo;
    double storedValue1;
    double storedValue2;

    @Override
    public void runOpMode() throws InterruptedException {
        thisCrazyServo = hardwareMap.get(ServoImplEx.class, "clawServo");

        thisCrazyServo.setPwmRange(new PwmControl.PwmRange(500, 2500));



//        if (gamepad1.left_bumper) {
//            storedValue1 = gamepad1.left_stick_y;
//        }
//        if (gamepad1.right_bumper) {
//            storedValue1 = gamepad1.left_stick_y;
//        }

        waitForStart();

        while (opModeIsActive()) {
            thisCrazyServo.setPosition(gamepad1.left_stick_y);
            telemetry.addData("Position", thisCrazyServo.getPosition());
//            telemetry.addData("Value 1", storedValue1!=nu?storedValue1:"undefined");
//            telemetry.addData("Value 2", thisCrazyServo.getPosition());
            telemetry.update();
        }
    }

}
