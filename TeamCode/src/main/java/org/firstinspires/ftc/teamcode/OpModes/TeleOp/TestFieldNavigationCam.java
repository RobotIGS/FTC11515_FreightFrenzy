package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.FieldNavigationCamera;

@TeleOp
public class TestFieldNavigationCam extends BaseTeleOp {
    GyroHardwareMap gyro;
    WebcamName camera;
    FieldNavigationCamera navi;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        gyro = new GyroHardwareMap(hardwareMap);
        gyro.init(hardwareMap);
        camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        navi = new FieldNavigationCamera(robot, gyro,camera, 0,0,0,0,0,0, 0,0,0,0.6/180,0.6);
    }

    @Override
    public void loop() {
        if (!navi.drive) {
            if (gamepad1.a) {
                navi.drive_to_pos(0,0,0.4,1);
            } else if (gamepad1.dpad_down) {
                navi.set_targetRotation(navi.rotation_y);
            } else {
                navi.drive_setSpeed(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x / 2, 0.6);
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
        telemetry.addData("image visible :", navi.targetVisible);
        telemetry.update();
    }
    }
