package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw extends SubsystemBase {


    private ServoImplEx claw;
    private ServoImplEx angleServo;


    //Servos for the linkages

    public Claw(HardwareMap hardwareMap){
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(ServoImplEx.class, "clawServo");
        angleServo = hardwareMap.get(ServoImplEx.class, "angleServo");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        angleServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        open();
        angleDown();
    }

    public void close(){
        claw.setPosition(0.69);
    }

    public void open(){
        claw.setPosition(0.5);
    }

    public void openMore(){
        claw.setPosition(0.8);
    }

    public void angleDown(){
        angleServo.setPosition(0.36);
    }

    public void angleUp(){
        angleServo.setPosition(0.55);
    }

    public void angleMore(){
        angleServo.setPosition(0.32);
    }




}
