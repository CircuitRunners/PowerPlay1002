package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class ProfiledLiftPositionCommand extends CommandBase {

    private PIDFController liftController;
    private TrapezoidProfile profile;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.026, 0.0055, 0.0013);
    public static double kV = 0.0011;
    public static double kA = 0.0;
    public static double kStatic = 0.01;

    private double tolerance = 5;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;
    public static double setpointPosError = 0;
    public static double setpointAccel;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd) {
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);

        liftController = new PIDFController(coefficients, kV, kA, 0.0, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.17;
            else if (liftPosition < 580) kG = 0.191;
            else kG = 0.218;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.85, 0.95);
    }


    @Override
    public void initialize() {

        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(700, 700),
                new TrapezoidProfile.State(targetPosition, 0),
                new TrapezoidProfile.State(lift.getLiftPosition(), lift.getLiftVelocity())
        );

        timer.reset();
        liftController.reset();
    }

    @Override
    public void execute() {
        liftPosition = lift.getLiftPosition();
        double currentVelo = lift.getLiftVelocity();
        double currentTime = timer.seconds();
        TrapezoidProfile.State state = profile.calculate(currentTime);


        //try to calculate acceleration setpoint
        setpointAccel = (state.velocity - setpointVel) / (currentTime - lastTime);

        //Update the real controller target
        liftController.setTargetPosition(state.position);
        liftController.setTargetVelocity(state.velocity);

        //Get the controller output
        double controllerOutput = liftController.update(liftPosition, currentVelo);

        //Apply kstatic
        controllerOutput += Math.signum(state.velocity) * kStatic;

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = state.position;
        setpointVel = state.velocity;
        setpointPosError = targetPosition - liftPosition;
        lastTime = currentTime;
    }

    @Override
    public boolean isFinished() {
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (holdAtEnd) lift.setLiftPower(0.2);
        else lift.stop();
    }



}
