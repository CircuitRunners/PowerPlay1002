package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw extends SubsystemBase {

    public enum ClawPosition {
        CLOSE(0.966), //0.962345 has worked
        OPEN(0.89),
        FULL_OPEN(0.64);

        public double position;

        ClawPosition(double position){
            this.position = position;
        }
    }

    private double poleGuideOpen = 0.75;
    private double poleGuideClose = 0.481;

    private ClawPosition clawPosition;


    private ServoImplEx claw;
    private ServoImplEx poleGuide;


    //Servos for the linkages

    public Claw(HardwareMap hardwareMap){
        //Retrieve servos from the hardware map
        claw = hardwareMap.get(ServoImplEx.class, "clawServo");
        poleGuide = hardwareMap.get(ServoImplEx.class, "poleGuide");

        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Experimental
        poleGuide.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        fullOpen();
        clawPosition = ClawPosition.FULL_OPEN;
        sheathPoleGuide();
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

    public void sheathPoleGuide(){
        poleGuide.setPosition(poleGuideOpen);
    }

    public void primePoleGuide(){
        poleGuide.setPosition(poleGuideClose);
    }




}
