
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;

@Autonomous
public class FullBlueAutonomous extends FullAutonomous {
    @Override
    public ColorEnum getAllianceColor() {
        return ColorEnum.Blue;
    }
}
