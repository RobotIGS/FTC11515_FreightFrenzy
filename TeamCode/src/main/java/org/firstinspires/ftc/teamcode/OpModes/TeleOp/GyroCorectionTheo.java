package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class GyroCorectionTheo extends BaseTeleOp {
    double gyroPosition;
    double angleRel;
    double vx;
    double vy;
    double vr;
    GyroHardwareMap hwgy;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        hwgy = new GyroHardwareMap(hardwareMap);
        hwgy.init(hardwareMap);
    }

    private double f(double da) {
        // todo: replace this (lambda)
        // min is ca. 0.05 ( +da < 10)
        // max is ca. 0.6 (+da < 180)
        if (da < 0) {
            da = -da;
        }
        da = (da/180)*0.6 + 0.05;
        return da;
    }

    public void loop() {
        Orientation angles;
        angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gyroPosition += 1.0 * (gamepad1.left_trigger - gamepad1.right_trigger);
        angleRel = gyroPosition - angles.firstAngle;

        if (angleRel > 180) {
            angleRel -= 180;
        } else if (angleRel < -180) {
            angleRel += 180;
        }

        telemetry.addData("valuesFirstAngle", angles.firstAngle);
        telemetry.addData("angleRel", angleRel);

        vy = 0.3 * -gamepad1.left_stick_y;
        vx = 0.3 * gamepad1.left_stick_x;
        vr = f(angleRel);

        if (gamepad1.y) {
            gyroPosition = angles.firstAngle;
        }

        if ((angleRel > -4) && (angleRel < 4)) {
            omniWheel.setMotors(vy, vx, 0);
        } else if (angleRel > 0) {
            omniWheel.setMotors(vy, vx, -vr);
        } else if (angleRel < 0) {
            omniWheel.setMotors(vy, vx, vr);
        }
        omniWheel.setMotors(vy,vx,0);
    }
}
