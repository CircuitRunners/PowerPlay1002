package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    //Drive motors and list to hold them
    private DcMotorEx lf, lb, rf, rb;
    private List<DcMotorEx> motors;

    //IMU sensor
    private BNO055IMU imu;
    //Offset variable for resetting heading;
    private double headingOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Retrieve them from the hardware map
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //Add all the dt motors to the list
        motors = Arrays.asList(lf, lb, rf, rb);

        //Reverse right side motors
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //Set the zero power behavior to brake
        for (DcMotorEx motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Ensure all motors are set to no encoders (since there are none plugged in)
        for (DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Send line to telemetry indicating initialization is done
        telemetry.addLine("Ready for start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            //Read gamepad joysticks
            //Check the deadband of the controller
            double y = (abs(gamepad1.left_stick_y) > 0.02) ? -gamepad1.left_stick_y : 0.0; // Remember, this is reversed!
            double x = (abs(gamepad1.left_stick_x) > 0.02) ? gamepad1.left_stick_x * 1.1 : 0.0; // Counteract imperfect strafing
            double rx = (abs(gamepad1.right_stick_x) > 0.02) ? gamepad1.right_stick_x : 0.0;

            //Read heading and subtract offset, then renormalize
            double heading = AngleUnit.normalizeRadians(-imu.getAngularOrientation().firstAngle - headingOffset);

            //Reset the zero point for field centric by making the current heading the offset
            if(gamepad1.b){
                headingOffset += heading;
                //Vibrate the gamepad
                gamepad1.rumble(0.0, 1.0, 300);
            }

            y = cubeInput(y, 0.2);
            x = cubeInput(x, 0.2);
            rx = cubeInput(rx, 0.2);


            double rotX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotY = x * Math.sin(heading) + y * Math.cos(heading);

            x = rotX;
            y = rotY;

            //Find motor powers
            double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * 0.69;
            double backLeftPower = (y - x + rx) / denominator * 0.69;
            double frontRightPower = (y - x - rx) / denominator * 0.69;
            double backRightPower = (y + x - rx) / denominator * 0.69;

            //Set motor powers
            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);

            telemetry.addData("Heading", heading);
            telemetry.addLine("Press B on Gamepad 1 to reset heading");
            telemetry.update();

        }

        //Stop all motors
        for(DcMotorEx motor : motors) motor.setPower(0);

    }

    private static double cubeInput(double input, double factor) {
        double t = factor * Math.pow(input, 3);
        double r = input * (1 - factor);
        return t + r;
    }
}
