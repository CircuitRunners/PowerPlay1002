package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class ProfiledLiftPositionCommand extends CommandBase {

    private ProfiledPIDController liftController;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.043, 0.025, 0.001);
    private double kG = 0.17; //gravity
    private double tolerance = 5;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition){
        this(lift, targetPosition, false);
    }

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd){
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        //Add a feedforward term to counteract gravity
        liftController = new ProfiledPIDController(coefficients.kP, coefficients.kI, coefficients.kD,
                new TrapezoidProfile.Constraints(400, 430));
        liftController.setTolerance(tolerance);
    }
    @Override
    public void initialize(){
        //once
        lift.stop();
        liftController.reset();
        liftController.setGoal(targetPosition);
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        //Update the lift power with the controller
        lift.setLiftPower(liftController.calculate(liftPosition) + kG);
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if(holdAtEnd) lift.setLiftPower(0.18);
        else lift.stop();
    }

}
