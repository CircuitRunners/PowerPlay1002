package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Lift

@Config
@TeleOp(name="PID Tuner")
class LiftPIDTuner : CommandOpMode() {


    private lateinit var lift: Lift
    private lateinit var claw: Claw

    private var target = 0.0


    override fun initialize() {

        lift = Lift(hardwareMap)
        claw = Claw(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        val driver = GamepadEx(gamepad1)

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        SequentialCommandGroup(
                                InstantCommand({target = 3000.0}),
                                LiftPositionCommand(lift, 3000),
                                WaitCommand(1000),
                                InstantCommand({target = 0.0}),
                                LiftPositionCommand(lift, 0)
                        )
                )
    }

    override fun run() {
        super.run()

        telemetry.addData("Target", target)
        telemetry.addData("Actual", lift.liftPosition)

        telemetry.update()
    }
}