package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw{

    private HardwareMap hardwareMap;

    private Servo claw;

    //Servos for the linkages


    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void init(){
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(Servo.class, "clawServo");
    }

    public void clampClose(){
        claw.setPosition(1);
    }
    public void clampOpen(){
        claw.setPosition(0);
    }




}
