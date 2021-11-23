package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class GyroCorection extends BaseTeleOp {
    double gyroPosition;
    double angleRel;
    GyroHardwareMap hwgy;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        hwgy = new GyroHardwareMap(hardwareMap);
        hwgy.init(hardwareMap);
    }

    public void loop() {
        Orientation angles;
        angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angleRel = gyroPosition - angles.firstAngle;

        if (angleRel > 180) {
            angleRel -= 180;
        } else if (angleRel < -180) {
            angleRel += 180;
        }

        telemetry.addData("valuesFirstAngle", angles.firstAngle);
        telemetry.addData("angleRel", angleRel);

        if (gamepad1.y) {
            gyroPosition = angles.firstAngle;
        }
        if (gamepad1.x) {
            if ((angleRel > -10) && (angleRel < 10)) {
                omniWheel.setMotors(0, 0, 0);
            } else if (angleRel > 0) {
                omniWheel.setMotors(0, 0, -0.2);
            } else if (angleRel < 0) {
                omniWheel.setMotors(0, 0, 0.2);
            }
        }
    }
}
