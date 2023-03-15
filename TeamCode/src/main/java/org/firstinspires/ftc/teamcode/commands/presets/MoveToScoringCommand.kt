package org.firstinspires.ftc.teamcode.commands.presets

import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Arm
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.commands.presets.MoveToScoringCommand.Presets
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand
import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftPositionCommand
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmPositions
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftPositions

class MoveToScoringCommand(lift: Lift, arm: Arm, claw: Claw, preset: Presets) : ParallelCommandGroup() {

    enum class Presets {
        GROUND,
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
                when (preset) {
                    Presets.GROUND -> InstantCommand({}) //do nothing
                    Presets.SHORT ->
                        ProfiledLiftPositionCommand(lift, LiftPositions.SHORT.position.toDouble(), true)
                    Presets.MID ->
                        ProfiledLiftPositionCommand(lift, LiftPositions.MID.position.toDouble(), true)
                    Presets.HIGH ->
                        ProfiledLiftPositionCommand(lift, LiftPositions.HIGH.position.toDouble(), true)

                },

                InstantCommand({
                    when (preset) {
                        Presets.GROUND -> arm.position = ArmPositions.GROUND.position
                        Presets.SHORT, Presets.MID, Presets.HIGH -> arm.position = ArmPositions.SCORING.position
                    }
                }),

                InstantCommand({
                    if(claw.clawPosition == Claw.ClawPosition.FULL_OPEN) claw.close()
                }),
        )
        addRequirements(lift)
    }
}