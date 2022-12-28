package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw extends SubsystemBase {

    private HardwareMap hardwareMap;

    private ServoImplEx claw;
    private ServoImplEx angleServo;

    //Servos for the linkages

    public Claw(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(ServoImplEx.class, "clawServo");
        angleServo = hardwareMap.get(ServoImplEx.class, "angleServo");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        angleServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
