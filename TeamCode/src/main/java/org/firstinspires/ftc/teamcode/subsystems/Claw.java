package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw extends SubsystemBase {

    public enum ClawPosition {
        CLOSE(0.217),
        OPEN(0.165),
        FULL_OPEN(0.03);

        public double position;

        ClawPosition(double position){
            this.position = position;
        }
    }

    private ClawPosition clawPosition;


    private ServoImplEx claw;
    private ServoImplEx angleServo;


    //Servos for the linkages

    public Claw(HardwareMap hardwareMap){
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(ServoImplEx.class, "clawServo");
        angleServo = hardwareMap.get(ServoImplEx.class, "angleServo");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
        angleServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        fullOpen();
        clawPosition = ClawPosition.FULL_OPEN;
        angleDown();
    }

    public ClawPosition getClawPosition(){
        return clawPosition;
    }

    public void close(){
        claw.setPosition(ClawPosition.CLOSE.position);
        clawPosition = ClawPosition.CLOSE;
    }

    public void open(){
        claw.setPosition(ClawPosition.OPEN.position);
        clawPosition = ClawPosition.OPEN;
    }

    public void fullOpen(){
        claw.setPosition(ClawPosition.FULL_OPEN.position);
        clawPosition = ClawPosition.FULL_OPEN;
    }

    public void angleDown(){
        angleServo.setPosition(0.413);
    }

    public void angleUp(){
        angleServo.setPosition(0.62);
    }

    public void angleMore(){
        angleServo.setPosition(0.62);
    }




}
