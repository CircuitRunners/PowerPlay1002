package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    //Drive motors and list to hold them
    private DcMotorEx lf, lb, rf, rb;
    private List<DcMotorEx> motors;

//    //IMU sensor
//    private BNO055IMU imu;
//    //Offset variable for resetting heading;
//    private double headingOffset = 0;

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

        //Set the zero power behavior to brake
        for (DcMotorEx motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Send line to telemetry indicating initialization is done
        telemetry.addLine("Ready for start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            //Read gamepad joysticks

            //refer to gm0 for mecanum math

        }

        //Stop all motors
        for(DcMotorEx motor : motors) motor.setPower(0);


    }
}
