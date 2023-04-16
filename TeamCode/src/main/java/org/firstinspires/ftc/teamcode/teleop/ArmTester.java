package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class ArmTester extends CommandOpMode {

    private double stepValue = 0.1;
    private double pos = 0;

    ServoImplEx leftArm, rightArm;

    @Override
    public void initialize() {
        leftArm = hardwareMap.get(ServoImplEx.class, "leftArm");
        rightArm = hardwareMap.get(ServoImplEx.class, "rightArm");


        //Set the maximum pwm range
        leftArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        GamepadEx driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                (()->{ stepValue /= 10; })
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                (()->{ stepValue *= 10; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                (()->{ if (pos <= 1 - stepValue) pos += stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                (()->{ if (pos >= 0 + stepValue) pos -= stepValue; })
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                (()->{ if (pos <= 1 - (10 * stepValue)) pos += 10*stepValue; })
        );
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                (()->{ if (pos >= 0 + 10*stepValue) pos -= 10*stepValue; })
        );

    }
    @Override
    public void run() {
        super.run();

        if ((Math.abs(gamepad1.left_stick_y) > 0.01) && gamepad1.square) pos = gamepad1.left_stick_y;

        rightArm.setPosition(pos);
        leftArm.setPosition(pos);

        telemetry.addData("Position", rightArm.getPosition());
        telemetry.addData("Position", leftArm.getPosition());
        telemetry.addData("Step Value", stepValue);
        telemetry.addData("Step Value", gamepad1.left_stick_y);

        telemetry.update();
    }

}
