package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

class DriveBase(hardwareMap: HardwareMap): SubsystemBase() {

    private val lf: DcMotorEx
    private val lb: DcMotorEx
    private val rf: DcMotorEx
    private val rb: DcMotorEx

    private val motorList: List<DcMotorEx>

    private var previousPowers = arrayListOf(0.0, 0.0, 0.0, 0.0)
    private val minChange = 0.005


    init {
        lf = hardwareMap[DcMotorEx::class.java, "lf"]
        lb = hardwareMap[DcMotorEx::class.java, "lb"]
        rf = hardwareMap[DcMotorEx::class.java, "rf"]
        rb = hardwareMap[DcMotorEx::class.java, "rb"]

        motorList = listOf(lf, lb, rf, rb)

        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE


        motorList.forEach {it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER}
        motorList.forEach {it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE}
    }


    /**
     * Array in the order of lf, lb, rf, rb.
     *
     */
    fun setMotorPowers(powers: Array<Double>){
        for(i in 0..4){
            if(abs(powers[i] - previousPowers[i]) >= minChange){
                motorList[i].power = powers[i]
                previousPowers[i] = powers[i]
            }
        }
    }

    fun stop(){
        setMotorPowers(arrayOf(0.0, 0.0, 0.0, 0.0))
    }





}