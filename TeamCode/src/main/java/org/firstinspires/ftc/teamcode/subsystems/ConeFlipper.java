package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ConeFlipper extends SubsystemBase {

    public enum FlipperPosition {
        UP(0.8345037465095519),
        DOWN(0.49955332746505734),
        UP_POSITION(0.8419147591580881);

        public double position;

        FlipperPosition(double position){
            this.position = position;
        }
    }

    private FlipperPosition flipperPosition;

    private ServoImplEx coneFlipper;

    // Servos for the linkages

    public ConeFlipper(HardwareMap hardwareMap){
        //Retrieve servos from the hardware map
        coneFlipper = hardwareMap.get(ServoImplEx.class, "coneFlipper");
        coneFlipper.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Experimental
        coneFlipper.setPwmRange(new PwmControl.PwmRange(500, 2500));

        upPosition();
        flipperPosition = FlipperPosition.UP_POSITION;
    }

//    public FlipperPosition getFlipperPosition(){
//        return flipperPosition;
//    }

    public void down(){
        coneFlipper.setPosition(FlipperPosition.DOWN.position);
        flipperPosition = FlipperPosition.DOWN;
    }

    public void up(){
        coneFlipper.setPosition(FlipperPosition.UP.position);
        flipperPosition = FlipperPosition.UP;
    }

    public void upPosition() {
        coneFlipper.setPosition(FlipperPosition.UP_POSITION.position);
        flipperPosition = FlipperPosition.UP_POSITION;
    }
}
