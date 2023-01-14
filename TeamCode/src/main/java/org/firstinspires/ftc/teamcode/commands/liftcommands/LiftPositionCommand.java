package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.031, 0, 0.002);
    private double kG = 0.176; //gravity
    //Bottom 0.14, 0.145, 0.176
    private double tolerance = 3;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public LiftPositionCommand(Lift lift, int targetPosition){
        this(lift, targetPosition, false);
    }

    public LiftPositionCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        addRequirements(lift);


        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        //Add a feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0, 0, 0.12, (x, v) -> {
            if(x < 283) return 0.128;
            else if(x < 580) return 0.137;
            else return 0.148;
        });
        liftController.setOutputBounds(-0.9, 0.95);
    }
    @Override
    public void initialize(){
        //once
        lift.stop();
        liftController.reset();
        liftController.setTargetPosition(targetPosition);
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        //Update the lift power with the controller
        lift.setLiftPower(liftController.update(liftPosition));
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) lift.setLiftPower(0.18);
        else lift.stop();
    }

}
