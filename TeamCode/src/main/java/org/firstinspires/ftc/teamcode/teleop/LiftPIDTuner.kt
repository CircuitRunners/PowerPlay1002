package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand
import org.firstinspires.ftc.teamcode.commands.RetractLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Lift

@Config
@Autonomous(name="PID Tuner")
class LiftPIDTuner() : CommandOpMode() {


    private lateinit var lift: Lift
    private lateinit var claw: Claw



    override fun initialize() {
        super.init()

        lift = Lift(hardwareMap)
        claw = Claw(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)


        schedule(SequentialCommandGroup(
                LiftPositionCommand(lift, 3000.0),
                RetractLiftCommand(lift, claw)
        )
        )
    }

    override fun run() {
        super.run()

        telemetry.addData("Target", 3000.0)
        telemetry.addData("Actual", lift.liftPosition)

        telemetry.update()
    }
}