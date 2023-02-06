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
    public static PIDCoefficients coefficients = new PIDCoefficients(0.032, 0.05, 0.002);
    //Bottom 0.14, 0.145, 0.176
    private double tolerance = 4;
    private boolean holdAtEnd;
    private final Lift lift;

    public static double targetPosition;

    public static double liftPosition = 0;
    public static double liftVelocity = 0;

    public LiftPositionCommand(Lift lift, int targetPosition, boolean holdAtEnd){
        addRequirements(lift);


        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        //Add a feedforward term to counteract gravity
        liftController = new PIDFController(coefficients, 0.0012, 0.0, 0.02, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.175;
            else if (liftPosition < 580) kG = 0.195;
            else kG = 0.218;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.65, 0.95);
    }
    @Override
    public void initialize(){
        //once
        liftController.reset();
        liftController.setTargetPosition(targetPosition);
    }

    //Run repeatedly while the command is active
    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        liftVelocity = lift.getLiftVelocity();

        //Update the lift power with the controller
        lift.setLiftPower(liftController.update(liftPosition, liftVelocity));
    }

    @Override
    public boolean isFinished(){
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted){
        if (holdAtEnd) lift.setLiftPower(0.2);
        else lift.stop();
    }

}
