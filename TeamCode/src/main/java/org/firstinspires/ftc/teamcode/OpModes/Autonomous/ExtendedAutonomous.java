package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.PositionEnum;

public abstract class ExtendedAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public void run() {
        driveToCarousel();
        rotateCarousel();
        driveToShippingHub();
        placeElementAtPosition(PositionEnum.Bottom);
        driveToWall();
        // not working !!!
        // parkInWarehouse();
    }
}
