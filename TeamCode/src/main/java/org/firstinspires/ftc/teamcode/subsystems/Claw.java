package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw{

    private HardwareMap hardwareMap;

    private Servo clawIntake;

    //Servos for the linkages


    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");

        //Retrieve servos from the hardware map
    }

    public void clampClose(){
        clawIntake.setPosition(0);

    }
    public void clampOpen(){
        clawIntake.setPosition(1);
    }




}
