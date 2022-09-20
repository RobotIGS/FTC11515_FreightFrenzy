package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareMaps.MotorHardwareMap;

@TeleOp
public class MotorTest extends BaseTeleOp {
    @Override
    public void initialize() {
        robot = new MotorHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.motor_front_left.setPower(gamepad1.left_stick_y);
        } else {
            robot.motor_front_left.setPower(0);
        }
    }
}
