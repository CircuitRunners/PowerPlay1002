package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp
@Disabled
public class ServoTest extends CommandOpMode {

    private Arm arm;

    @Override
    public void initialize(){

        arm = new Arm(hardwareMap);


        schedule(
                new InstantCommand(() -> arm.setLevel(Arm.ArmPositions.HIGH))
        );
    }


}
