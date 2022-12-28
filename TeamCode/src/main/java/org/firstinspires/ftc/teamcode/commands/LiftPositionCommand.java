package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.03, 0, 0);
//    private double kStatic = 0.1; //gravity
    private double tolerance;
    private boolean holdAtEnd = false;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public LiftPositionCommand(Lift lift, double targetPosition){
        this(lift, targetPosition, 10);
    }

    public LiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd){
        this(lift, targetPosition, 10);
        this.holdAtEnd = holdAtEnd;
    }

    public LiftPositionCommand(Lift lift, double targetPosition, double tolerance){
        this.lift = lift;
        this.tolerance = tolerance;
        this.targetPosition = targetPosition;

        liftController = new PIDFController(coefficients);
        liftController.setOutputBounds(-0.7, 1);
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
        return Math.abs(liftPosition - targetPosition) < tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if(holdAtEnd) lift.setLiftPower(0.1);
        else lift.stop();
    }

}
