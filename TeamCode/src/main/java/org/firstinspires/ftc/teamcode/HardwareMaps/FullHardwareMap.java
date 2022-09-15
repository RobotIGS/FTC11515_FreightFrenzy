package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FullHardwareMap extends BaseHardwareMap {
    public FullHardwareMap(HardwareMap hwMap) {
        super(hwMap);
    }

    @Override
    public void init(HardwareMap hwMap) {
        motor_front_left = hwMap.get(DcMotor.class, "hub1_motorport0");
        motor_front_right = hwMap.get(DcMotor.class, "hub1_motorport2");
        motor_rear_left = hwMap.get(DcMotor.class, "hub1_motorport3");
        motor_rear_right = hwMap.get(DcMotor.class, "hub1_motorport1");
        /*
        motor_shovel = hwMap.get(DcMotor.class, "hub2_motorport1");
        motor_lift = hwMap.get(DcMotor.class, "hub2_motorport0");
        motor_carousel = hwMap.get(DcMotor.class, "hub2_motorport2");

        distanceSensor_front_mid = hwMap.get(DistanceSensor.class, "hub1_i2c2");
        distanceSensor_right = hwMap.get(DistanceSensor.class, "hub2_i2c1");
        distanceSensor_left = hwMap.get(DistanceSensor.class, "hub2_i2c2");
        distanceSensor_carousel = hwMap.get(DistanceSensor.class, "hub1_i2c3");
        colorSensor_down = hwMap.get(ColorSensor.class, "hub1_i2c1");

        motor_front_left.setDirection(DcMotor.Direction.FORWARD);
        motor_front_right.setDirection(DcMotor.Direction.REVERSE);
        motor_rear_left.setDirection(DcMotor.Direction.FORWARD);
        motor_rear_right.setDirection(DcMotor.Direction.REVERSE);
        motor_shovel.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_lift.setDirection(DcMotorSimple.Direction.FORWARD);

        motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         */
    }
}
