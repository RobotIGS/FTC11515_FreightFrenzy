package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.BarcodeEnum;

public abstract class FullAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public void run() {
        BarcodeEnum barcodePosition = detectBarcodePosition();
        driveToCarousel();
        rotateCarousel();
        driveToShippingHub();
        placeElementAtPosition(barcodePosition);
        driveToWall();
        parkInWarehouse();
    }
}
