package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.OmniWheel;

public abstract class BaseTeleOp extends OpMode {
    BaseHardwareMap robot;
    OmniWheel omniWheel;

    @Override
    public void init() {
        initialize();

        omniWheel = new OmniWheel(robot);
    }

    public abstract void initialize();
}
