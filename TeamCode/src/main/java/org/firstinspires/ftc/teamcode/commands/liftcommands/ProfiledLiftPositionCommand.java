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

    public static PIDCoefficients coefficients = new PIDCoefficients(0.03045, 0.00443, 0.00065);
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0.0; //0.14

    private double tolerance = 4;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastTime = 0;


    public ProfiledLiftPositionCommand(Lift lift, double targetPosition) {
        this(lift, targetPosition, false);
    }

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd) {
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);

        liftController = new PIDFController(coefficients, kV, kA, kStatic, (x, v) -> {
            double kG = 0;
            if (liftPosition < 283) kG = 0.13;
            else if (liftPosition < 580) kG = 0.14;
            else kG = 0.15;

            return kG * 12 / lift.getVoltage();
        });
        liftController.setOutputBounds(-0.84, 0.95);

        profileController = new ProfiledPIDController(0, 0, 0,
                new TrapezoidProfile.Constraints(690, 740));

    }


    @Override
    public void initialize() {
        //once
        timer.reset();

        profileController.reset(lift.getLiftPosition(), lift.getLiftVelocity());
        profileController.setGoal(targetPosition);
        profileController.setTolerance(3);

        liftController.reset();
    }

    @Override
    public void execute() {
        liftPosition = lift.getLiftPosition();

        //Update the profile (ignore the output)
        profileController.calculate(liftPosition);

        double acceleration = (profileController.getSetpoint().velocity - lastVelocity) / (timer.seconds() - lastTime);

        //Update the real controller target
        liftController.setTargetPosition(profileController.getSetpoint().position);

        //Get the controller output
        double controllerOutput = liftController.update(liftPosition);

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = profileController.getSetpoint().position;
        setpointVel = profileController.getSetpoint().velocity;

        lastVelocity = profileController.getSetpoint().velocity;
        lastTime = timer.seconds();
    }

    @Override
    public boolean isFinished() {
        //End if the lift position is within the tolerance
        return Math.abs(targetPosition - liftPosition) <= tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (holdAtEnd) lift.setLiftPower(0.188);
        else lift.stop();
    }



}
