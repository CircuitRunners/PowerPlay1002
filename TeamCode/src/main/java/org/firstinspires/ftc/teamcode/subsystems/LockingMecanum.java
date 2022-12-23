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
        //@TODO Replace 1 with the position of the servo where it is locked
        leftLinkage.setPosition(0.23);
        rightLinkage.setPosition(0.6);
    }

    //Move servos to unlock wheels
    public void unlock(){
        //@TODO Replace 0 with the position of the servo where it is unlocked
        leftLinkage.setPosition(0.05);
        rightLinkage.setPosition(0.95);
    }




}
