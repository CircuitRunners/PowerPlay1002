package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.commands.BulkCacheCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    
    //Drive motors and list to hold them
    private DcMotorEx lf, lb, rf, rb;
    //IMU sensor
    private BNO055IMU imu;
    private Claw claw;
    private Lift lift;
    private Intake intake;
    //Offset variable for resetting heading;
    private double headingOffset = 0;
    private boolean prevHeadingReset = false;


    //    boolean clawButtonPrev = false;
//    boolean clawClosed = false;

    @Override
    public void initialize() {

        //Set the bulk cache command to continuously run
        schedule(new BulkCacheCommand(hardwareMap));

        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
        claw.clampOpen();
//        intake.closeArms();

        //Retrieve dt motors from the hardware map
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //Add all the dt motors to the list
        List<DcMotorEx> motors = Arrays.asList(lf, lb, rf, rb);

        for (DcMotorEx motor : motors) {
            //Set the zero power behavior to brake
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Ensure all motors are set to no encoders
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Reverse left side motors
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //Controllers
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx manipulator = new GamepadEx(gamepad2);

        //control to rnu intake whenever the trigger is pressed
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    intake.intake();
                })
                .whenInactive(() -> {
                    intake.stop();
                });

        //control to outtake whenever right trigger is pressed (for safety)
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(() -> {
                    intake.intake();
                })
                .whenInactive(() -> {
                    intake.stop();
                });


        //TODO: toggle control for the claw
        manipulator.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenActive(claw::clampClose, claw::clampOpen);

        //Control the intake arms (manually for now)
//        manipulator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .toggleWhenActive(intake::openArms, intake::closeArms);


        //Send line to telemetry indicating initialization is done
        telemetry.addLine("Ready for start!");
        telemetry.update();

    }

    @Override
    public void run() {
        //Run the other functions in the superclass
        super.run();

        //Read heading and subtract offset, then normalize again
        double heading =
                imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;


        //Reset the zero point for field centric by making the current heading the offset
        if (gamepad1.back && !prevHeadingReset) {
            headingOffset += heading;
            //Vibrate the gamepad
            gamepad1.rumble(0.0, 1.0, 300);
        }
        prevHeadingReset = gamepad1.back;


        //Read gamepad joysticks
        //Check the deadband of the controller
        double y = (abs(gamepad1.left_stick_y) > 0.02) ? -gamepad1.left_stick_y : 0.0; // Remember, this is reversed!
        double x = (abs(gamepad1.left_stick_x) > 0.02) ? -gamepad1.left_stick_x * 1.1 : 0.0; // Counteract imperfect strafing
        double rx = (abs(gamepad1.right_stick_x) > 0.02) ? gamepad1.right_stick_x : 0.0;

        boolean groundLevel = (gamepad2.dpad_left);

        //Apply a curve to the inputs
        y = cubeInput(y, 0.2);
        x = cubeInput(x, 0.2);
        rx = cubeInput(rx, 0.2);

        //Make a vector out of the x and y and rotate it by the heading
        Vector2d vec = new Vector2d(x, y);
        vec = vec.rotated(heading - headingOffset);
        x = vec.getX();
        y = vec.getY();

        //Ensure powers are in the range of [-1, 1] and set power
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1.0);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //Set motor powers
        lf.setPower(frontLeftPower);
        lb.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rb.setPower(backRightPower);

        telemetry.addData("Current Heading with offset", "%.2f", AngleUnit.DEGREES.fromRadians(heading));
        telemetry.addData("Offset", "%.2f", AngleUnit.DEGREES.fromRadians(headingOffset));
        telemetry.addLine("Press \"back\" on Gamepad 1 to reset heading");
        telemetry.update();

        // Intake goes brr
        if (gamepad2.dpad_up) {
            lift.goToPosition(lift.level3,100);
        }
        else if (gamepad2.dpad_right) {
            lift.goToPosition(lift.level2,100);
        }
        else if (gamepad2.dpad_down) {
            lift.goToPosition(lift.level1,100);
        }

        if (gamepad2.y) {
            claw.clampClose();
        }
        if (gamepad2.b) {
            claw.clampOpen();
            lift.goToLowerLimit();
        }
        if (gamepad2.x) lift.stop();

        if(groundLevel == true){

        }


    }

    private static double cubeInput(double input, double factor) {
        double t = factor * Math.pow(input, 3);
        double r = input * (1 - factor);
        return t + r;
    }
}
