package org.firstinspires.ftc.teamcode.Tools;

import java.lang.reflect.Parameter;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FieldNavigation {
    // robot object
    BaseHardwareMap robot;
    GyroHardwareMap gyro;

    // statics
    private static final double COUNTS_PER_MOTOR_REV = 751.8; // 223rpm
    //private static final double COUNTS_PER_MOTOR_REV = 384.5; // 435rpm
    private static final double DRIVE_GEAR_REDUCTION = 1.0;      // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_CMS = 10.0;     // For figuring circumference
    private static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    // robot stuff (constants)
    private static final double lx = 1;
    private static final double lz = 1;

    // last steps of the motors
    private double last_steps_fl;
    private double last_steps_fr;
    private double last_steps_rl;
    private double last_steps_rr;

    // robot speeds
    private double vx = 0;
    private double vz = 0;
    private double wy = 0;

    // position of the robot
    public double position_x;
    public double position_z;
    public double rotation_y;

    // wheel speeds
    private double wheelSpeeds[] = {0,0,0,0};

    // drive flags
    private boolean drive = false;
    private boolean target_reached = false;

    // target position for navigation
    private double target_position_x;
    private double target_position_z;
    private double drive_speed;
    private double drive_acc;

    // target rotation of the robot
    private double target_rotation_y;
    private double start_rotation_y;
    private double gyro_start_rotation;

    /* constructor */

    /**
     * one class to rule them all, (for the navigation of the robot)
     * @param robot BaseHardwareMap object
     * @param gyro  GyroHardwareMap object
     * @param x     x start location
     * @param z     z start location
     * @param ry    start y rotation
     */
    public FieldNavigation(BaseHardwareMap robot, GyroHardwareMap gyro, double x, double z, double ry) {
        // hardware
        this.robot = robot;
        this.gyro = gyro;

        // set start rotation (gyro)
        gyro_start_rotation = gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // set start steps
        last_steps_fl = robot.motor_front_left.getCurrentPosition();
        last_steps_fr = robot.motor_front_right.getCurrentPosition();
        last_steps_rl = robot.motor_rear_left.getCurrentPosition();
        last_steps_rr = robot.motor_rear_right.getCurrentPosition();

        // robot position and rotation
        position_x = x;
        position_z = z;
        rotation_y = ry;
        start_rotation_y = ry;
   }

    /**
     * calculate the speed of the motors out of the speed from the robot
     * @param vx spped in x direction
     * @param vz spped in y direction
     * @param wy rotation speed (0)
     * @return an array with motor speeds (4x1) (front left, front right, rear left, rear right)
     */
    private void calculateWheelSpeeds(double vx, double vz, double wy) {
        wheelSpeeds[0] = vx + -vz - (lx+lz)*wy;
        wheelSpeeds[1] = vx - -vz + (lx+lz)*wy;
        wheelSpeeds[2] = vx - -vz - (lx+lz)*wy;
        wheelSpeeds[3] = vx + -vz + (lx+lz)*wy;
    }

    /**
     * calculate wheel step targets
     * @param wheelSpeeds an array with the speeds of the motors (return from calculateWheelSpeeds)
     * @param maxDistance the absolute maximum distance of x and z in cm
     * @return an array with the motor targets (4x1) (front left, front right, rear left, rear right)
     */
    private double[] calculateWheelTargets(double[] wheelSpeeds, double maxDistance) {
        // relative speed * steps for max Distance
        double[] wheelTargets = {
            robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0] * (COUNTS_PER_CM * maxDistance),
            robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1] * (COUNTS_PER_CM * maxDistance),
            robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2] * (COUNTS_PER_CM * maxDistance),
            robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3] * (COUNTS_PER_CM * maxDistance)
        };

        return wheelTargets;
    }

    /**
     * convert distance (field cord system) into distance rel to robot
     * @param dx distance in x direction relative to the robot (field cord system)
     * @param dz distance in z direction relative to the robot (field cord system)
     * @return   an Array [0]: distance in x, [1]: distance in z
     */
    private double[] convert_pos2rel(double dx, double dz) {
        double d = Math.sqrt(Math.pow(dx,2) + Math.pow(dz,2));
        if (d == 0) {
            double ret[] = {0,0};
            return ret;
        }

        double alpha = Math.asin(dz/d) - Math.toRadians(rotation_y);
        double[] ret = {
            Math.cos(alpha) * d,
            Math.sin(alpha) * d
        };

        return ret;
    }

    /**
     * convert rel distance from robot to distance (field cord system) rel to robot
     * @param dx distance in x direction relative to the robot
     * @param dz distance in z direction relative to the robot
     * @return   an Array [0]: distance in x, [1]: distance in z
     */
    private double[] convert_rel2pos(double dx, double dz) {
        double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dz,2));
        if (d == 0) {
            double ret[] = {0,0};
            return ret;
        }

        double alpha = Math.asin(dz/d) + Math.toRadians(rotation_y);
        double[] ret = {
            Math.cos(alpha)*d,
            Math.sin(alpha)*d
        };

        return ret;
    }

    /**
     * drive a specified distance relative to the robot current position and rotation with a specified accuracy
     * @param dx    distance in x direction
     * @param dz    distance in z direction
     * @param speed speed
     * @param acc   accuracy in cm
     */
    public void drive_rel(double dx, double dz, double speed, double acc) {
        // convert rel2pos
        double[] pos = convert_rel2pos(dx,dz);

        // set target pos
        target_position_x = pos[0];
        target_position_z = pos[1];

        // set drive flag and target acc
        drive = true;
        drive_speed = speed;
        drive_acc = acc;
        target_reached = false;
    }

    /**
     * drive to a coordinate on the field with a specified accuracy
     * @param x     x coordinate
     * @param z     z coordinate
     * @param speed speed
     * @param acc   accuracy in cm
     */
    public void drive_to_pos(double x, double z, double speed, double acc) {
        // set target pos
        target_position_x = x;
        target_position_z = z;

        // set drive stuff
        drive = true;
        drive_speed = speed;
        drive_acc = acc;
        target_reached = false;
    }

    /**
     * @param vx    speed in x direction
     * @param vz    speed in z direction
     * @param wy    rotation speed of the robot
     * @param speed speed factor between -1 and 1
     */
    public void drive_setMotors(double vx, double vz, double wy, double speed) {
        if (drive) {
            drive_stop();
        }

        // make sure the speeds are between -1 and 1
        if (Math.abs(speed) > 1) {
            speed /= Math.abs(speed);
        }
        if (Math.abs(vx) > 1) {
            vx /= Math.abs(vx);
        }
        if (Math.abs(vz) > 1) {
            vx /= Math.abs(vz);
        }
        if (Math.abs(wy) > 1) {
            vx /= Math.abs(wy);
        }

        // get wheel speeds
        calculateWheelSpeeds(vx, vz, wy);

        // set motor power
        robot.motor_front_left.setPower(wheelSpeeds[0]*speed);
        robot.motor_front_right.setPower(wheelSpeeds[1]*speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2]*speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3]*speed);
    }

    /**
     * stop controlled drive
     */
    public void drive_stop() {
        // overwrite vx and vz
        vx = 0;
        vz = 0;

        // Stop all motion;
        drive_setMotors(0, 0, 0, 0);

        // set drive flag
        drive = false;
    }

    /**
     * test if the driving target was reached
     * @return a Boolean indecating if the target was reached
     */
    public boolean target_reached() {
        double d = Math.sqrt(Math.pow(target_position_x - position_x, 2) + Math.pow(target_position_z - position_z, 2));
        if (d <= Math.abs(drive_acc)) {
            // overwrite flags
            drive = false;
            target_reached = true;

            // overwrite vx and vz
            vx = 0;
            vz = 0;
        }
        return target_reached;
    }

    /**
     * PID Regulator
     */
    private void PID() {
        double error = target_rotation_y - rotation_y;
        wy = error/180 * 0.2;   // P
        wy += 0;                // I
        wy += 0;                // D
    }

    protected void stepPos() {
        // get delta steps
        double delta_s1 = last_steps_fl - robot.motor_front_left.getCurrentPosition();
        double delta_s2 = last_steps_fr - robot.motor_front_right.getCurrentPosition();
        double delta_s3 = last_steps_rl - robot.motor_rear_left.getCurrentPosition();
        double delta_s4 = last_steps_rr - robot.motor_rear_right.getCurrentPosition();

        // calculate the distance
        double dx = (delta_s1 + delta_s2 + delta_s3 + delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);
        double dz = (-delta_s1 + delta_s2 + delta_s3 - delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);

        // set new position
        double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dz,2));
        if (d == 0) {
            return;
        }
        double alpha = Math.asin(dz/d) + Math.toRadians(rotation_y);

        this.position_x -= Math.cos(alpha)*d;
        this.position_z -= Math.sin(alpha)*d;

        // set new last steps for next calculations
        last_steps_fl -= delta_s1;
        last_steps_fr -= delta_s2;
        last_steps_rl -= delta_s3;
        last_steps_rr -= delta_s4;
    }

    protected void stepDrive() {
        if (drive && !target_reached()) {
            // get distance to target
            vx = target_position_x - position_x;
            vz = target_position_z - position_z;

            // transform pos2rel
            double[] rel = convert_pos2rel(vx,vz);
            vx = rel[0];
            vz = rel[1];

            // get max distance
            double maxDistance = Math.max(Math.abs(vx), Math.abs(vz));

            // get direction speeds in percent
            vx /= maxDistance;
            vz /= maxDistance;
        }
        // set motor speeds
        drive_setMotors(vx, vz, wy, drive_speed);
    }

    protected void stepGyro() {
        // get rotation based on the start rotation
        rotation_y = gyro_start_rotation - gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + start_rotation_y;

        if (rotation_y < -180) {
            rotation_y += 180 *(rotation_y % 180);
        } else if (rotation_y > 180) {
            rotation_y -= 180 *(rotation_y % 180);
        }

        this.PID();
    }

    /**
     * go through every step methode
     */
    public void step() {
        stepGyro();
        stepDrive();
        stepPos();
    }
}
