package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw{

    private HardwareMap hardwareMap;

    private Servo claw;
    private Servo angleServo;

    //Servos for the linkages

    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(Servo.class, "clawServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        clampOpen();
        angleDown();
    }

    public void clampClose(){
        claw.setPosition(0.2);
    }
    public void clampOpen(){
        claw.setPosition(0);
    }

    public void angleDown(){
        angleServo.setPosition(0);
    }

    public void angleUp(){
        angleServo.setPosition(0.3);
    }




}
