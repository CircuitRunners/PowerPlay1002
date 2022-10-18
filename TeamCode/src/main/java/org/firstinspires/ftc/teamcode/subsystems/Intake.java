package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    private HardwareMap hardwareMap;

    private DcMotorSimple leftMotor;
    private DcMotorSimple rightMotor;

    private Servo leftServo;
    private Servo rightServo;


    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    //Called in init
    public void init(){
        leftMotor = hardwareMap.get(DcMotorSimple.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorSimple.class, "rightMotor");

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    @Override
    public void periodic(){

    }
}
