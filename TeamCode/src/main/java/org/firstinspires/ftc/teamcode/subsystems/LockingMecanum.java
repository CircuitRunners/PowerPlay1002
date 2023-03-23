package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LockingMecanum extends SubsystemBase {


    //Servos for the linkages
    private Servo leftLinkage;
    private Servo rightLinkage;


    public LockingMecanum(HardwareMap hardwareMap){
        leftLinkage = hardwareMap.get(Servo.class, "leftLmec");
        rightLinkage = hardwareMap.get(Servo.class, "rightLmec");

        unlock();

    }

    //Move servos to lock wheels
    public void lock(){
        leftLinkage.setPosition(0.25);
        rightLinkage.setPosition(0.6);
    }

    //Move servos to unlock wheels
    public void unlock(){
        leftLinkage.setPosition(0.045);
        rightLinkage.setPosition(0.95);
    }




}
