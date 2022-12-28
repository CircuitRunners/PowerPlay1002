package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {


    private DcMotorSimple leftMotor;
    private DcMotorSimple rightMotor;



    public Intake(HardwareMap hardwareMap){

        leftMotor = hardwareMap.get(DcMotorSimple.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorSimple.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void intake(){
        leftMotor.setPower(0.7);
        rightMotor.setPower(0.7);
    }

    public void outtake(){
        leftMotor.setPower(-0.7);
        rightMotor.setPower(-0.7);
    }

    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

}
