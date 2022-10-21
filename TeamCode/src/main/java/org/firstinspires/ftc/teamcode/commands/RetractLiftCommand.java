package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class RetractLiftCommand extends ParallelCommandGroup {

    public RetractLiftCommand(Lift lift, Claw claw){
        addCommands(
                new LiftPositionCommand(lift, 0, 7),
                new InstantCommand(
                        () -> {
                            claw.clampOpen();
                        }
                )
        );
    }

}
