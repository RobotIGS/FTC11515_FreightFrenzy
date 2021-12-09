package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.BaseAutonomous;

public class ControlledDrive {
    static final double COUNTS_PER_MOTOR_REV = 751.8;    // 223rpm
//    static final double COUNTS_PER_MOTOR_REV = 384.5;    // 435rpm
    static final double DRIVE_GEAR_REDUCTION = 1.0;      // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CMS = 10.0;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    BaseHardwareMap robot;
    BaseAutonomous autonomous;

    public ControlledDrive(BaseHardwareMap robot, BaseAutonomous autonomous) {
        this.robot = robot;
        this.autonomous = autonomous;
    }

    public void drive(double distanceForward, double distanceSideways, double speed) {
        start(distanceForward, distanceSideways, speed);
        while (autonomous.opModeIsActive() && (robot.motor_front_left.isBusy() ||
                robot.motor_front_right.isBusy() || robot.motor_rear_left.isBusy() ||
                robot.motor_rear_right.isBusy())) {}
        stop();
    }

    public void start(double distanceForward, double distanceSideways, double speed) {
        double maxDistance = Math.max(Math.abs(distanceForward), Math.abs(distanceSideways));

        double[] wheelSpeeds = OmniWheel.calculateWheelSpeeds(distanceForward / maxDistance,
                distanceSideways / maxDistance, 0).toArray();

        // Determine new target position
        double[] targets = {
                robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0] * (COUNTS_PER_CM * maxDistance),
                robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1] * (COUNTS_PER_CM * maxDistance),
                robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2] * (COUNTS_PER_CM * maxDistance),
                robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3] * (COUNTS_PER_CM * maxDistance)
        };

        // And pass to motor controller
        robot.motor_front_left.setTargetPosition((int) targets[0]);
        robot.motor_front_right.setTargetPosition((int) targets[1]);
        robot.motor_rear_left.setTargetPosition((int) targets[2]);
        robot.motor_rear_right.setTargetPosition((int) targets[3]);

        // Turn On RUN_TO_POSITION
        robot.motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.motor_front_left.setPower(wheelSpeeds[0] * speed);
        robot.motor_front_right.setPower(wheelSpeeds[1] * speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2] * speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3] * speed);
    }

    public void stop() {
        // Stop all motion;
        robot.motor_front_left.setPower(0);
        robot.motor_front_right.setPower(0);
        robot.motor_rear_left.setPower(0);
        robot.motor_rear_right.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motor_front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
