package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class ProfiledLiftPositionCommand extends CommandBase {

    private ProfiledPIDController profileController;
    private PIDFController liftController;

    public static PIDCoefficients coefficients = new PIDCoefficients(0.02, 0.005, 0.0012);
    public static double kV = 0.00155;
    public static double kA = 0.0021;
    public static double kStatic = 0.025;
    private double voltageFactor = 1.0;

    private double tolerance = 5;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;
    public static double setpointPosError = 0;
    public static double acceleration;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastTime = 0;

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd) {
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);

        voltageFactor = 12.0 / lift.getVoltage();

        liftController = new PIDFController(coefficients, kV, kA, kStatic, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.178;
            else if (liftPosition < 580) kG = 0.191;
            else kG = 0.215;

            return kG * voltageFactor;
        });
        liftController.setOutputBounds(-0.85, 0.95);

        profileController = new ProfiledPIDController(0, 0, 0,
                new TrapezoidProfile.Constraints(770, 770));

    }


    @Override
    public void initialize() {
        //once
        timer.reset();

        profileController.reset(lift.getLiftPosition(), lift.getLiftVelocity());
        profileController.setGoal(targetPosition);
        profileController.setTolerance(0, 0);

        liftController.reset();
    }

    @Override
    public void execute() {
        liftPosition = lift.getLiftPosition();
        double currentVelo = lift.getLiftVelocity();

        //Update the profile (ignore the output)
        profileController.calculate(liftPosition);

        //Update the real controller target
        liftController.setTargetPosition(profileController.getSetpoint().position);
        liftController.setTargetVelocity(profileController.getSetpoint().velocity);

        //Get the controller output
        double controllerOutput = liftController.update(liftPosition, currentVelo);

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        acceleration = (currentVelo - lastVelocity) / (timer.seconds() - lastTime);
        setpointPos = profileController.getSetpoint().position;
        setpointVel = profileController.getSetpoint().velocity;
        setpointPosError = profileController.getPositionError();
        lastVelocity = currentVelo;
        lastTime = timer.seconds();
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
