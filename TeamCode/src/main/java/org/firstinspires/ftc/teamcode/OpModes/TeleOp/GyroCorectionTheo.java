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
    double angleError;
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

    private double P_Regler(double ek) { // ek = error; PID Regler ohne ID
        if (ek < 0) { ek = -ek; }

        double v = ek/180 *0.6;

        return v;
    }

    public void loop() {
        Orientation angles;
        angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angleError = gyroPosition - angles.firstAngle;

        if (angleError > 180) {
            angleError -= 180;
        } else if (angleError < -180) {
            angleError += 180;
        }

        telemetry.addData("valuesFirstAngle", angles.firstAngle);
        telemetry.addData("angleError", angleError);
        telemetry.addData("vr", vr);

        vy = 0.3 * -gamepad1.left_stick_y;
        vx = 0.3 * gamepad1.left_stick_x;
        vr = P_Regler(angleError);

        if (gamepad1.left_trigger > 0) {
            omniWheel.setMotors(vy, vx, -gamepad1.left_trigger * 0.5);
            gyroPosition = angles.firstAngle;
        } else if (gamepad1.right_trigger > 0) {
            omniWheel.setMotors(vy, vx, gamepad1.right_trigger * 0.5);
            gyroPosition = angles.firstAngle;
        } else if (angleError > 0) {
            omniWheel.setMotors(vy, vx, -vr);
        } else if (angleError <= 0) {
            omniWheel.setMotors(vy, vx, vr);
        }
    }
}
