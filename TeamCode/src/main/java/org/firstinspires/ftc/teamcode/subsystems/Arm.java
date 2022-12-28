package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm extends SubsystemBase {

    public enum ArmPositions {
        DOWN(0.15),
        SHORT(0.7),
        MID(0.8),
        HIGH(0.75);

        public double position;

        ArmPositions(double position){
            this.position = position;
        }
    }


    private ServoImplEx leftServo;
    private ServoImplEx rightServo;


    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.9, 0.85);
    private TrapezoidProfile armProfile =
            new TrapezoidProfile(constraints, new TrapezoidProfile.State(ArmPositions.DOWN.position, 0),
                    new TrapezoidProfile.State(ArmPositions.DOWN.position, 0)
            );

    private ElapsedTime timer = new ElapsedTime();



    public Arm(HardwareMap hardwareMap){
    
        leftServo = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightServo = hardwareMap.get(ServoImplEx.class, "rightArm");

        leftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
        setPosition(ArmPositions.DOWN.position);
    }

    public void forceDown(){
        leftServo.setPosition(0.15);
        rightServo.setPosition(0.15);
    }

    //Set a preset level
    public void setLevel(ArmPositions level){
        setPosition(level.position);
    }


}
