package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LockingMecanum extends SubsystemBase {

    private HardwareMap hardwareMap;

    //Servos for the linkages
    private Servo leftLinkage;
    private Servo rightLinkage;


    public LockingMecanum(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        //@TODO is the check for position necessary or should it set the position to 0 everytime regardless?
        if (leftLinkage.getPosition()!=0){
            leftLinkage.setPosition(0);
        }
    }

    //Move servos to lock wheels
    public void lock(){
        //@TODO Replace 1 with the position of the servo where it is locked
        leftLinkage.setPosition(1);
        rightLinkage.setPosition(1);
    }

    //Move servos to unlock wheels
    public void unlock(){
        //@TODO Replace 0 with the position of the servo where it is unlocked
        leftLinkage.setPosition(0);
        rightLinkage.setPosition(0);
    }




}
