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

    }

    public void lock(){
        //Move servos to lock wheels
    }

    public void unlock(){
        //Move servos to unlock wheels
    }




}
