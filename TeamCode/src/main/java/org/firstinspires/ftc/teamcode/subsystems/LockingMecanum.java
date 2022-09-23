package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LockingMecanum {

    private HardwareMap hardwareMap;

    //Servos for the linkages


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
