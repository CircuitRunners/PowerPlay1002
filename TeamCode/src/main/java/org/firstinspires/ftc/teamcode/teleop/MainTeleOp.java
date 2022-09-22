package org.firstinspires.ftc.teamcode.teleop;

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

        //Add all the dt motors to the list
        motors = Arrays.asList(lf, lb, rf, rb);

        //Retrieve them from the hardware map
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //Reverse right side motors
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

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
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            //Read heading and subtract offset, then renormalize
            double heading = AngleUnit.normalizeRadians(-imu.getAngularOrientation().firstAngle - headingOffset);

            //Reset the zero point for field centric by making the current heading the offset
            if(gamepad1.b){
                headingOffset = heading;
                //Vibrate the gamepad
                gamepad1.rumble(300);
            }

            //Find motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

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
}
