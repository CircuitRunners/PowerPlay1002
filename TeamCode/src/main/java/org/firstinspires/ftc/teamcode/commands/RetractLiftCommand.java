package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

//Brings everything to rest position
public class RetractLiftCommand extends ParallelCommandGroup {

    public RetractLiftCommand(Lift lift, Arm arm, Claw claw){
        addCommands(
                new LiftPositionCommand(lift, 0, 3)
        );
    }

}
