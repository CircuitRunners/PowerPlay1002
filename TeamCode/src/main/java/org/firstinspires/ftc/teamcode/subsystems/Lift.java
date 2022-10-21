package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift extends SubsystemBase {

    private HardwareMap hardwareMap;

    private DcMotorEx liftMotor;

    private PIDFController liftController =
            new PIDFController(new PIDCoefficients(0.05, 0, 0),0,0, 0);


    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }


    public void init(){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void periodic(){

    }

    public void up(){
        liftMotor.setPower(0.4);
    }

    public void down(){
        liftMotor.setPower(-0.2);
    }

    public void hold(){
        liftMotor.setPower(0.1);
    }

}
