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

        leftMotor = hardwareMap.get(DcMotorSimple.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorSimple.class, "rightMotor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    public void intake(){
        leftMotor.setPower(0.7);
        rightMotor.setPower(0.7);
    }

    public void outtake(){
        leftMotor.setPower(-0.7);
        rightMotor.setPower(-0.7);
    }

    public void stop(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void openArms(){
        leftServo.setPosition(1);
        rightServo.setPosition(0);
    }

    public void closeArms(){
        leftServo.setPosition(0.74);
        rightServo.setPosition(0.3);
    }
    //@TODO add thing that detects whether cone can go in and close or open it based on its dimensions

}
