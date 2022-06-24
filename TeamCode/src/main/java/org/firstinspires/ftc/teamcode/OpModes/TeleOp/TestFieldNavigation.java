
package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class TestFieldNavigation extends BaseTeleOp {
    private GyroHardwareMap gyro;
    private FieldNavigation navi;

    boolean started = false;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        gyro.init(hardwareMap);
        navi = new FieldNavigation(robot,gyro, 0,0,0);
    }

    @Override
    public void loop() {
        navi.drive_setMotors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x/2, 0.4);

        navi.step();

        telemetry.addData("pos x:", navi.position_x);
        telemetry.addData("pos z:", navi.position_z);
        telemetry.update();
    }
}
