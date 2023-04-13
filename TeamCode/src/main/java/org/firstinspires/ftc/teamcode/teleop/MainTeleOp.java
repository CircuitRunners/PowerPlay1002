package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ManualLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand;
import org.firstinspires.ftc.teamcode.commands.presets.RetractOuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ConeFlipper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LockingMecanum;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTeleOp extends CommandOpMode {

    private LockingMecanum lockingMecanum;
    private Lift lift;
    private Claw claw;
    private Arm arm;
    private ConeFlipper flipper;

//    Drive motors and list to hold them
    private DcMotorEx lf, lb, rf, rb;
//    IMU sensor
    private BNO055IMU imu;
//    Offset variable for resetting heading;
    private double headingOffset = toRadians(-90);
    private boolean prevHeadingReset = false;
    private boolean lmecOn = false;

    private ManualLiftCommand manualLiftCommand;
    private ManualLiftResetCommand manualLiftResetCommand;

    @Override
    public void initialize() {
//        PhotonCore.enable();
//        Set the bulk cache command to continuously run
        schedule(new BulkCacheCommand(hardwareMap));

        lockingMecanum = new LockingMecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        flipper = new ConeFlipper(hardwareMap);
//        intake = new Intake(hardwareMap);


//        Retrieve dt motors from the hardware map
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

//        Add all the dt motors to the list
        List<DcMotorEx> motors = Arrays.asList(lf, lb, rf, rb);

        for (DcMotorEx motor : motors) {
//            Set the zero power behavior to brake
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            Ensure all motors are set to no encoders
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

//        Reverse left side motors
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

//        Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

//        Commands

//        Set up gamepads
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

//        Lmec Control
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7)
                .whenActive(() -> {
                    lockingMecanum.lock();
                    lmecOn = true;
                })
                .whenInactive(() -> {
                    lockingMecanum.unlock();
                    lmecOn = false;
                });

        //Set up commands
        manualLiftCommand = new ManualLiftCommand(lift, manipulator);
        manualLiftResetCommand = new ManualLiftResetCommand(lift, manipulator);

        lift.setDefaultCommand(new PerpetualCommand(manualLiftCommand));

//        ConeFlipper control
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenActive(flipper::down, flipper::up);

        //Claw control
        manipulator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenActive(claw::close, () -> {
                    if(arm.canFullOpenClaw()) claw.fullOpen();
                    else claw.open();
                });

//        //Manual pole guide control
//        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .toggleWhenActive(claw::sheathPoleGuide, claw::primePoleGuide);

        //Bottom limit lift reset
        manipulator.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(manualLiftResetCommand);

        //High preset
        new Trigger(() -> manipulator.getLeftY() > 0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.HIGH)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Mid preset
        new Trigger(() -> manipulator.getRightY() > -0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.MID)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Short preset
        new Trigger(() -> manipulator.getRightY() < 0.4)
                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.SHORT)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));

        //Ground (terminal dropping) arm preset
//        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
//                .whenActive(new MoveToScoringCommand(lift, arm, claw, MoveToScoringCommand.Presets.GROUND).withTimeout(2000));

        //Full retract preset
        new Trigger(() -> manipulator.getLeftY() < -0.6)
                .whenActive(new RetractOuttakeCommand(lift, arm, claw)
                        .withTimeout(1900)
                        .interruptOn(() -> manualLiftCommand.isManualActive()));




        //Send line to telemetry indicating initialization is done
        telemetry.addLine("Ready for start!");
        telemetry.update();

    }

    int loopNum = 0;
    @Override
    public void run() {

        //Run the other functions in the superclass
        super.run();

        if(loopNum == 0){
            claw.fullOpen();
            loopNum++;
        }

        //Read heading and subtract offset, then normalize again
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //If lmec is on, force robot centric control
        double heading = AngleUnit.normalizeRadians(orientation.firstAngle - headingOffset);

        //Reset the zero point for field centric by making the current heading the offset
        if (gamepad1.x && !prevHeadingReset) {
            headingOffset += heading;
            gamepad1.rumble(0.0, 1.0, 300);
        }

        prevHeadingReset = gamepad1.x;
//
        if (arm.getPosition() > 0.251) {
            claw.primePoleGuide();
        } else {
            claw.sheathPoleGuide();
        }

        //Read gamepad joysticks
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.06; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        //Apply a curve to the inputs
        y = cubeInput(y, 0.3);
        x = cubeInput(x, 0.3);
        rx = cubeInput(rx, 0.3);

        //Make a vector out of the x and y and rotate it by the heading
        if(!lmecOn){
            Vector2d vec = new Vector2d(x, y).rotated(-heading);
            x = (lmecOn) ? 0 : vec.getX();
            y = vec.getY();
        }

        //Ensure powers are in the range of [-1, 1] and set power
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //Set motor powers


        lf.setPower((Math.signum(frontLeftPower) * 0.03) + frontLeftPower);
        lb.setPower((Math.signum(backLeftPower) * 0.03) + backLeftPower);
        rf.setPower((Math.signum(frontRightPower) * 0.03) + frontRightPower);
        rb.setPower((Math.signum(backRightPower) * 0.03) + backRightPower);


        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Current Heading with offset", "%.2f", AngleUnit.DEGREES.fromRadians(heading));
        telemetry.addData("Offset", "%.2f", AngleUnit.DEGREES.fromRadians(headingOffset));
        telemetry.addLine("Press Square on Gamepad 1 to reset heading");
//        telemetry.addData("Z deg", AngleUnit.DEGREES.fromRadians(orientation.firstAngle));
//        telemetry.addData("Y deg", AngleUnit.DEGREES.fromRadians(orientation.secondAngle));
//        telemetry.addData("X deg", AngleUnit.DEGREES.fromRadians(orientation.thirdAngle));
        telemetry.addData("Lift Position", lift.getLiftPosition());
        telemetry.addData("can open full claw", arm.canFullOpenClaw());

        packet.put("Lift Position", lift.getLiftPosition());
        packet.put("Lift velocity", lift.getLiftVelocity());
        packet.put("Lift controller output", LiftPositionCommand.controllerOutput);
        packet.put("Lift target position", LiftPositionCommand.setpointPos);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();

    }

    private static double cubeInput(double input, double factor) {
        double t = factor * Math.pow(input, 3);
        double r = input * (1 - factor);
        return t + r;
    }
}
