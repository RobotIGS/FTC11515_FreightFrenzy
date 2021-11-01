package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;

public class OmniWheelResults {
    double front_left;
    double front_right;
    double rear_left;
    double rear_right;

    public OmniWheelResults(double front_left, double front_right, double rear_left, double rear_right) {
        this.front_left = front_left;
        this.front_right = front_right;
        this.rear_left = rear_left;
        this.rear_right = rear_right;
    }

    public OmniWheelResults(double[] speeds) {
        if (speeds.length != 4) throw new IllegalArgumentException("The array must be exactly of size 4");

        this.front_left = speeds[0];
        this.front_right = speeds[1];
        this.rear_left = speeds[2];
        this.rear_right = speeds[3];
    }

    public void setMotors(BaseHardwareMap robot) {
        robot.motor_front_left.setPower(front_left);
        robot.motor_front_right.setPower(front_right);
        robot.motor_rear_left.setPower(rear_left);
        robot.motor_rear_right.setPower(rear_right);
    }
}
