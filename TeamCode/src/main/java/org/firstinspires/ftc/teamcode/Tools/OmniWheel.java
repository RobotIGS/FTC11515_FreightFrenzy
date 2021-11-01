package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;

public class OmniWheel {
    BaseHardwareMap robot;

    public OmniWheel(BaseHardwareMap robot) {
        this.robot = robot;
    }

    /**
     * Calculate omniwheel speeds, see https://link.springer.com/content/pdf/10.1007%2F978-3-540-70534-5.pdf#page=159
     *
     * @param forwardVelocity   vehicle velocity in forward direction
     * @param sidewardsVelocity vehicle velocity in sideways direction
     * @param rotationVelocity  vehicle rotational velocity
     * @return a OmniWheelResults object containing the wheel speeds
     */
    public OmniWheelResults calculateWheelSpeeds(double forwardVelocity, double sidewardsVelocity, double rotationVelocity) {
        return new OmniWheelResults(
                forwardVelocity * -sidewardsVelocity * -rotationVelocity,
                forwardVelocity * sidewardsVelocity * rotationVelocity,
                forwardVelocity * sidewardsVelocity * -rotationVelocity,
                forwardVelocity * -sidewardsVelocity * rotationVelocity
        );
    }

    /**
     * Calculate wheel speeds and set motors to these values
     *
     * @param forwardVelocity   vehicle velocity in forward direction
     * @param sidewardsVelocity vehicle velocity in sideways direction
     * @param rotationVelocity  vehicle rotational velocity
     */
    public void setMotors(double forwardVelocity, double sidewardsVelocity, double rotationVelocity) {
        this.calculateWheelSpeeds(forwardVelocity, sidewardsVelocity, rotationVelocity).setMotors(robot);
    }
}

