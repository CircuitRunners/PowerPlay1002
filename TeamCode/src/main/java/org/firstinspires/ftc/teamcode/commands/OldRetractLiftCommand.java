package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class OldRetractLiftCommand extends ParallelCommandGroup {

    public OldRetractLiftCommand(Lift lift, Claw claw){
        addCommands(
                new LiftPositionCommand(lift, 0, 7),
                new InstantCommand(
                        claw::clampOpen
                )
        );
    }

}
