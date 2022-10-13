package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {


    private DcMotorEx liftMotor;

    private PIDFController liftController =
            new PIDFController(new PIDCoefficients(1, 0, 0),0,0, 0);


    
}
