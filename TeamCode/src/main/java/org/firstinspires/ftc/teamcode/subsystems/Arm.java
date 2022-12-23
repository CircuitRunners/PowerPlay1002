package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {

    private HardwareMap hardwareMap;

    private Servo leftServo;
    private Servo rightServo;


    public Arm(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

    
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public void down() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void up(){
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }
    //@TODO add thing that detects whether cone can go in and close or open it based on its dimensions

}
