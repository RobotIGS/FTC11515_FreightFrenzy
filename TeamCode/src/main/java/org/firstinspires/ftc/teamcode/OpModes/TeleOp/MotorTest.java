package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

@TeleOp
public class MotorTest extends BaseTeleOp {
    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            robot.motor_front_left.setPower(1);
        } else {
            robot.motor_front_left.setPower(0);
        }

        if (gamepad1.y) {
            robot.motor_front_right.setPower(1);
        } else {
            robot.motor_front_right.setPower(0);
        }

        if (gamepad1.a) {
            robot.motor_rear_left.setPower(1);
        } else {
            robot.motor_rear_left.setPower(0);
        }

        if (gamepad1.b) {
            robot.motor_rear_right.setPower(1);
        } else {
            robot.motor_rear_right.setPower(0);
        }
    }
}
