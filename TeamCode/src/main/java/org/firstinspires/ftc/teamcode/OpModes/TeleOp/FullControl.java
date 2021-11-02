package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

@TeleOp
public class FullControl extends BaseTeleOp {
    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        omniWheel.setMotors(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
