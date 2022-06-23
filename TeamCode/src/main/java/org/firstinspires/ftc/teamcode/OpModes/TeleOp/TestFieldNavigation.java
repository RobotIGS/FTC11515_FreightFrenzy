
package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;


@TeleOp
public class TestFieldNavigation extends BaseTeleOp {
     private FieldNavigation navi;

    boolean started = false;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        navi = new FieldNavigation(robot, 0,0,30);
    }

    @Override
    public void loop() {
        if (gamepad1.a && !started) {
            navi.drive_rel(200,100,0.2);
            started = true;
        }
        navi.step();

        telemetry.addData("pos x:", navi.position_x);
        telemetry.addData("pos z:", navi.position_z);
        telemetry.addData("target reached:", navi.drive_finished());
        telemetry.update();
    }
}
