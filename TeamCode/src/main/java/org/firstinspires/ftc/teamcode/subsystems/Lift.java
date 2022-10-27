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

    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void setLiftPower(double power){
        liftMotor.setPower(power);
    }

    public void stop(){
        liftMotor.setPower(0);
    }

    public double getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }

    public void resetLiftPosition(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean atUpperLimit(){
        return getLiftPosition() > 1000;
    }

    public boolean atLowerLimit(){
        return getLiftPosition() < 5;
    }
    //@TODO find ticks to inches conversion and add levels.
}
