package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class ProfiledLiftPositionCommand extends CommandBase {

    private ProfiledPIDController liftController;
    private ElevatorFeedforward feedforward;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.001, 0, 0);
    public static double kV = 0.00003;
    public static double kA = 0.006;
    public static double kStatic = 0.02;
    public static double kG = 0.17; //gravity

    private double tolerance = 3;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastTime = 0;


    public ProfiledLiftPositionCommand(Lift lift, double targetPosition){
        this(lift, targetPosition, false);
    }

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd){
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);

        liftController = new ProfiledPIDController(coefficients.kP, coefficients.kI, coefficients.kD,
                new TrapezoidProfile.Constraints(550, 550));
        feedforward = new ElevatorFeedforward(kStatic, kG, kV, kA);
    }


    @Override
    public void initialize(){
        //once
        timer.reset();

        liftController.reset(lift.getLiftPosition(), lift.getLiftVelocity());
        liftController.setGoal(targetPosition);
        liftController.setTolerance(3, 2);
    }

    @Override
    public void execute(){
        liftPosition = lift.getLiftPosition();
        double acceleration = (liftController.getSetpoint().velocity - lastVelocity) / (timer.seconds() - lastTime);

        double feedforwardOutput = feedforward.calculate(liftController.getSetpoint().velocity, acceleration);


        //Get the controller output and add the gravity feedforward
        double controllerOutput = liftController.calculate(liftPosition) + feedforwardOutput;

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = liftController.getSetpoint().position;
        setpointVel = liftController.getSetpoint().velocity;

        lastVelocity = liftController.getSetpoint().velocity;
        lastTime = timer.seconds();
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
