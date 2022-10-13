package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LockingMecanum {

    private HardwareMap hardwareMap;

    //Servos for the linkages
    private Servo leftLinkage;
    private Servo rightLinkage;


    public LockingMecanum(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        //Retrieve servos from the hardware map
    }

    public void lock(){
        //Move servos to lock wheels
    }

    public void unlock(){
        //Move servos to unlock wheels
    }




}
