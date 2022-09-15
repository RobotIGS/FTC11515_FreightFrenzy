package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;


@TeleOp
public class test extends BaseTeleOp {
    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            robot.motor_rear_right.setPower(-0.7);
            robot.motor_front_right.setPower(-0.7);
            robot.motor_rear_right.setPower(-0.7);
            robot.motor_front_right.setPower(-0.7);


        }
        else if (gamepad1.b){
            robot.motor_front_right.setPower(0.6);
            robot.motor_rear_right.setPower(0.4);
        }
        );
    }
}

