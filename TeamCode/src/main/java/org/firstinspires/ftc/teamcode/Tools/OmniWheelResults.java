package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;

import java.util.Arrays;

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

        maxValuesToOne();
    }

    public void setMotors(BaseHardwareMap robot) {
        robot.motor_front_left.setPower(front_left);
        robot.motor_front_right.setPower(front_right);
        robot.motor_rear_left.setPower(rear_left);
        robot.motor_rear_right.setPower(rear_right);
    }

    public double[] toArray() {
        return new double[]{front_left, front_right, rear_left, rear_right};
    }

    public void maxValuesToOne() {
        double[] speeds = toArray();

        for (int i = 0; i < speeds.length; i++) {
            speeds[i] = Math.abs(speeds[i]);
        }

        Arrays.sort(speeds);

        if (speeds[3] > 1) {
            front_left = front_left / speeds[3];
            front_right = front_right / speeds[3];
            rear_left = rear_left / speeds[3];
            rear_right = rear_right / speeds[3];
        }
    }
}
