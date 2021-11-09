package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

@TeleOp
public class GyroTest extends OpMode {
    GyroHardwareMap hwgy;
    @Override
    public void init() {

        hwgy = new GyroHardwareMap(hardwareMap);
        hwgy.init(hardwareMap);

    }

    @Override
    public void loop() {
        Orientation angles;
        angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("valuesAngleUnit" , angles.angleUnit);
        telemetry.addData("valuesAcquisitionTime" , angles.acquisitionTime);
        telemetry.addData("valuesFirstAngle" , angles.firstAngle);
        telemetry.addData("valuesSecondAngle" , angles.secondAngle);
        telemetry.addData("valuesThirdAngle" , angles.thirdAngle);
        telemetry.addData("valuesAxesOrder" , angles.axesOrder);
        telemetry.addData("valuesAxesReference" , angles.axesReference);
        telemetry.update();

    }
}
