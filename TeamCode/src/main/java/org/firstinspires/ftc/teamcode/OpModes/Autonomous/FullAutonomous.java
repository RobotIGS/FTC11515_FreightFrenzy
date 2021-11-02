package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

public abstract class FullAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public void run() {
        detectPositionBarcode();
        driveToCarousel();
        rotateCarousel();
        driveToShippingHub();
        placeElementAtPosition();
        driveToWall();
        parkInWarehouse();
    }
}
