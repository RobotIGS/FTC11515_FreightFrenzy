package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

public abstract class BasicAutonomous extends BaseAutonomous {
    @Override
    public BaseHardwareMap initializeHardwareMap() {
        return new FullHardwareMap(hardwareMap);
    }

    @Override
    public void run() {
        driveToCarousel();
        rotateCarousel();
        // parkInWarehouse_Basic(); This methode doesnt exist yet
    }
}
