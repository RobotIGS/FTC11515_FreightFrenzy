package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class TestFieldNavigation extends BaseTeleOp {
    GyroHardwareMap gyro;
    FieldNavigation navi;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        gyro.init(hardwareMap);
        navi = new FieldNavigation(robot,gyro, 0,0,0, 0.7/180,0.5);
    }

    @Override
    public void loop() {
        if (!navi.drive) {
            if (gamepad1.a) {
                navi.drive_to_pos(0,0,0.4,1);
            } else if (gamepad1.b) {
                navi.drive_to_pos(100,0,0.2,1);
            } else if (gamepad1.x) {
                navi.drive_to_pos(100, -100, 0.2, 1);
            } else if (gamepad1.y) {
                navi.drive_to_pos(0, -100, 0.2, 1);
            } else if (gamepad1.dpad_down) {
                navi.set_targetRotation(navi.rotation_y);
            } else {
                navi.drive_setSpeed(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x / 2, 0.6);
            }
        }

        navi.step();

        telemetry.addData("pos x:", navi.position_x);
        telemetry.addData("pos z:", navi.position_z);
        telemetry.addData("vx:", navi.vx);
        telemetry.addData("vz:", navi.vz);
        telemetry.addData("wy:", navi.wy);
        telemetry.addData("finished? :", navi.target_reached);
        telemetry.addData("rotation ", navi.rotation_y);
        telemetry.update();
    }
}
