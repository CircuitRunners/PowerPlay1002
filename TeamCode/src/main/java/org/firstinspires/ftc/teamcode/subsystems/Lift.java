package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift extends SubsystemBase {

    private HardwareMap hardwareMap;

    private DcMotorEx liftMotor;

    private Servo clawIntake;

    private PIDFController liftController =
            new PIDFController(new PIDCoefficients(1, 0, 0),0,0, 0);




    public Lift(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }


    public void init(){

    }

    @Override
    public void periodic(){

    }

}
