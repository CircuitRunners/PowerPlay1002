package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm extends SubsystemBase {

    public enum ArmPositions {
        SHORT(0.8),
        MID(0.7),
        HIGH(0.6);

        public double position;

        ArmPositions(double position){
            this.position = position;
        }
    }


    private Servo leftServo;
    private Servo rightServo;

    private TrapezoidProfile armProfile;
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.3, 0.3);

    private ElapsedTime timer = new ElapsedTime();


    public Arm(HardwareMap hardwareMap){
    
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        forceDown();

    }

    @Override
    public void periodic(){

        //Read the current target for the profile
        double position = armProfile.calculate(timer.seconds()).position;

        //Set servo positions
        leftServo.setPosition(position);
        rightServo.setPosition(position);

    }

    //Set the servos to a numerical position
    public void setPosition(double target) {

        //Create a new profile starting from the last position command
        armProfile = new TrapezoidProfile(
                constraints,
                new TrapezoidProfile.State(target, 0),
                new TrapezoidProfile.State(leftServo.getPosition(), 0)
        );

        //Reset the timer
        timer.reset();
    }

    //All the way to the rest position
    public void down(){
        setPosition(0);
    }

    public void forceDown(){
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    //Set a preset level
    public void setLevel(ArmPositions level){
        setPosition(level.position);
    }


}
