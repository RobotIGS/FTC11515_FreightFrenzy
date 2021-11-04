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

        telemetry.addData("values0" , angles.angleUnit);

        telemetry.addData("values1" , angles.acquisitionTime);

        telemetry.addData("values2" , angles.firstAngle);

        telemetry.addData("values4" , angles.secondAngle);

        telemetry.addData("values5" , angles.thirdAngle);

        telemetry.addData("values6" , angles.axesOrder);

        telemetry.addData("values7" , angles.axesReference);
        telemetry.update();

    }
}
