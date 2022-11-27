package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;


public class DropPreloadCommand extends ParallelCommandGroup {


    private SampleMecanumDrive drive;
    private Lift lift;
    private Claw claw;

    public DropPreloadCommand(SampleMecanumDrive drive, Lift lift, Claw claw){

        this.drive = drive;
        this.lift = lift;
        this.claw = claw;
    }
}
