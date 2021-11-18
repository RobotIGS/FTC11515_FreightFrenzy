package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;

@Autonomous
public class ControlledTestAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public ColorEnum getAllianceColor() {
        return null;
    }

    @Override
    public void run() {
        telemetry.addData("pos1", 0);
        telemetry.update();

        // 10cm nach vorne
        controlledDrive.drive(50, 0, 0.3);
        telemetry.addData("pos1", 1);
        telemetry.update();

        // 10cm nach rechts
        controlledDrive.drive(0, 50, 0.3);
        telemetry.addData("pos1", 2);
        telemetry.update();

        // 10cm nach hinten & 10cm nach links
        controlledDrive.drive(-50, 0, 0.3);
        telemetry.addData("pos1", 3);
        telemetry.update();

        controlledDrive.drive(0, -50, 0.3);
        telemetry.addData("pos1", 4);
        telemetry.update();
    }
}
