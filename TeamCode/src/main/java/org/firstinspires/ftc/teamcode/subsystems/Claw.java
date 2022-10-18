package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw{

    private HardwareMap hardwareMap;

    private Servo claw;

    //Servos for the linkages


    public Claw(HardwareMap hardwareMap){
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
