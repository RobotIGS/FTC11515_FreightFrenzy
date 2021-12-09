package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseHardwareMap {
    public HardwareMap hwMap;

    public DcMotor motor_front_right;
    public DcMotor motor_front_left;
    public DcMotor motor_rear_right;
    public DcMotor motor_rear_left;
    public DcMotor motor_shovel;
    public DcMotor motor_lift;
    public DcMotor motor_carousel;
    public DistanceSensor distanceSensor_front_mid;
    public DistanceSensor distanceSensor_right;
    public DistanceSensor distanceSensor_left;
    public DistanceSensor distanceSensor_carousel;
    public ColorSensor colorSensor_down;


    public BaseHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        init(hwMap);
    }

    public abstract void init(HardwareMap hwMap);
}
