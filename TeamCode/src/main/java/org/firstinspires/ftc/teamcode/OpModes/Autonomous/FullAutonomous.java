package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.BarcodeEnum;
import org.firstinspires.ftc.teamcode.Tools.PositionEnum;

public abstract class FullAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public void run() {
        // driveToCarousel(); dis sens right not worḱing
        rotateCarousel();
        driveToShippingHub();
        placeElementAtPosition(barcodePosition);
        driveToWall();
        parkInWarehouse();
    }
}
