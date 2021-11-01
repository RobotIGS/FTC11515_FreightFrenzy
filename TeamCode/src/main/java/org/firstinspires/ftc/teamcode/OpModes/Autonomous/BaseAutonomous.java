package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;

public abstract class BaseAutonomous extends LinearOpMode {
    BaseHardwareMap robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    void initialize() {
        robot = initializeHardwareMap();
        // TODO
    }

    public abstract BaseHardwareMap initializeHardwareMap();

    public abstract ColorEnum getAllianceColor();

    public abstract void run();

    public void driveToCarousel() {
        // TODO
    }

    public void rotateCarousel() {
        // TODO
    }

    public void parkInWarehouse() {
        // TODO
    }
}
