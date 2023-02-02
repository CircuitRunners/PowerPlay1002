package org.firstinspires.ftc.teamcode.commands.liftcommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class ProfiledLiftPositionCommand extends CommandBase {

    private PIDFController liftController;
//    private TrapezoidProfile profile;
    private MotionProfile profile2;


    public static PIDFController.PIDCoefficients coefficients =
            new PIDFController.PIDCoefficients();//p=0.0268, i=0.005, d=0.00147
    public static double kV = 0.00144;
    public static double kA = 0.00009;
    public static double kStatic = 0.04;

    private double tolerance = 5;
    private double integralThreshold = 20;
    private boolean holdAtEnd;
    private final Lift lift;
    private final double targetPosition;

    private double liftPosition;

    public static double setpointPos = 0;
    public static double setpointVel = 0;
    public static double setpointPosError = 0;
    public static double setpointAccel = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public ProfiledLiftPositionCommand(Lift lift, double targetPosition, boolean holdAtEnd) {
        this.holdAtEnd = holdAtEnd;
        this.lift = lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);

        coefficients.kP = 0.0268;
        coefficients.kI = 0.005;
        coefficients.kD = 0.00147;

        liftController = new PIDFController(coefficients, kV, kA, kStatic, (x, v) -> {
            double kG;
            if (liftPosition < 283) kG = 0.18;
            else if (liftPosition < 580) kG = 0.195;
            else kG = 0.22;

            return kG * lift.getVoltageComp();
        });
        liftController.setOutputBounds(-0.55, 0.95);
    }


    @Override
    public void initialize() {

//        profile = new TrapezoidProfile(
//                new TrapezoidProfile.Constraints(700, 700),
//                new TrapezoidProfile.State(targetPosition, 0),
//                new TrapezoidProfile.State(lift.getLiftPosition(), lift.getLiftVelocity())
//        );

        profile2 = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lift.getLiftPosition(), lift.getLiftVelocity()),
                new MotionState(targetPosition, 0),
                700,
                800,
                3000
        );

        timer.reset();
        liftController.reset();
    }

    @Override
    public void execute() {
        liftPosition = lift.getLiftPosition();
        double currentVelo = lift.getLiftVelocity();
        double currentTime = timer.seconds();
//        TrapezoidProfile.State state = profile.calculate(currentTime);
        MotionState state = profile2.get(currentTime);


        //Update the real controller target
        liftController.targetPosition = state.getX();
        liftController.targetVelocity = state.getV();
        liftController.targetAcceleration = state.getA();

        //Only use the integral gain if the lift is within a threshold of the target
        if(Math.abs(targetPosition - liftPosition) > integralThreshold) liftController.resetIntegralSum();

        //Get the controller output
        double controllerOutput = liftController.update(System.nanoTime(), liftPosition, currentVelo);

        //Update the lift power with the controller
        lift.setLiftPower(controllerOutput);

        setpointPos = state.getX();
        setpointVel = state.getV();
        setpointAccel = state.getA();
        setpointPosError = targetPosition - liftPosition;
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
