package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class CommonIssues extends LinearOpMode {



    DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;



    @Override
    public void runOpMode() throws InterruptedException {


        lf = hardwareMap.get(DcMotorEx .class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");


        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        lf.setPower(0.5);
        lb.setPower(0.5);
        rf.setPower(-0.5);
        rb.setPower(-0.5);

        sleep(1000);
//        ElapsedTime timer = new ElapsedTime();
//        while (timer.seconds() < 5 && opModeIsActive()){
//            //do nothing
//        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);



    }

}
