package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class FieldNavigation {
    // robot object
    protected BaseHardwareMap robot;
    protected GyroHardwareMap gyro;
    protected PIController rotation_pi_controller;

    // statics
    protected static final double COUNTS_PER_MOTOR_REV = 751.8; // 223rpm
    //protected static final double COUNTS_PER_MOTOR_REV = 384.5; // 435rpm
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;      // This is < 1.0 if geared UP
    protected static final double WHEEL_DIAMETER_CMS = 10.0;     // For figuring circumference
    protected static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);
    protected static final double ONE_D_R = 1 / (WHEEL_DIAMETER_CMS / 2);
    protected static final double R_D_FOUR = WHEEL_DIAMETER_CMS/8;
    protected static final double TWOPI_D_CPERMREV = (2*Math.PI) / COUNTS_PER_MOTOR_REV;

    // robot stuff (constants)
    protected static final double lx = 1;
    protected static final double lz = 1;

    // last steps of the motors
    private double last_steps_fl;
    private double last_steps_fr;
    private double last_steps_rl;
    private double last_steps_rr;

    private double[] gyro_correction_steps = {0,0,0,0};

    // robot speeds
    public double vx = 0;
    public double vz = 0;
    public double wy = 0;

    // position of the robot
    public double position_x;
    public double position_z;
    public double rotation_y;

    // wheel speeds
    private double[] wheelSpeeds = {0,0,0,0};

    // drive flags
    public boolean drive = false;
    public boolean target_reached = false;

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
     * @param PI_d  PI controller for gyro correction d value (MAXPWR / 180)
     * @param PI_i  PI controller for gyro correction i value
     */
    public FieldNavigation(BaseHardwareMap robot, GyroHardwareMap gyro, double x, double z, double ry, double PI_d, double PI_i) {
        // hardware
        this.robot = robot;
        this.gyro = gyro;

        // PI controller
        rotation_pi_controller = new PIController(PI_d, PI_i);

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
        target_rotation_y = ry;
   }

    /**
     * calculate the speed of the motors out of the speed from the robot
     * @param vx speed in x direction
     * @param vz speed in y direction
     * @param wy rotation speed
     */
    protected double[] calculateWheelSpeeds(double vx, double vz, double wy) {
        return new double[]{
                (ONE_D_R * vx) + (ONE_D_R * vz) - ((lx + lz) * wy * ONE_D_R),
                (ONE_D_R * vx) - (ONE_D_R * vz) + ((lx + lz) * wy * ONE_D_R),
                (ONE_D_R * vx) - (ONE_D_R * vz) - ((lx + lz) * wy * ONE_D_R),
                (ONE_D_R * vx) + (ONE_D_R * vz) + ((lx + lz) * wy * ONE_D_R)
        };
    }

    /**
     * calculate wheel step targets
     * @param wheelSpeeds an array with the speeds of the motors (return from calculateWheelSpeeds)
     * @param maxDistance the absolute maximum distance of x and z in cm
     * @return an array with the motor targets
     */
    protected double[] calculateWheelTargets(double[] wheelSpeeds, double maxDistance) {
        // relative speed * steps for max Distance

        return new double[]{
            robot.motor_front_left.getCurrentPosition() + wheelSpeeds[0] * (COUNTS_PER_CM * maxDistance),
            robot.motor_front_right.getCurrentPosition() + wheelSpeeds[1] * (COUNTS_PER_CM * maxDistance),
            robot.motor_rear_left.getCurrentPosition() + wheelSpeeds[2] * (COUNTS_PER_CM * maxDistance),
            robot.motor_rear_right.getCurrentPosition() + wheelSpeeds[3] * (COUNTS_PER_CM * maxDistance)
        };
    }

    /**
     * convert position (field cord system) into position (robot cord system)
     * @param dx x position (field cord system)
     * @param dz z position (field cord system)
     * @return   an Array with x and z position (robot cord system)
     */
    protected double[] convert_pos2rel(double dx, double dz) {
        double d = Math.sqrt(Math.pow(dx,2) + Math.pow(dz,2));
        double[] ret = {0,0};
        if (d == 0) {
            return ret;
        }

        double alpha = Math.asin(dz/d);
        if (dx < 0) {
            alpha = Math.PI - alpha;
        }
        alpha -= Math.toRadians(rotation_y);

        ret[0] = Math.cos(alpha) * d;
        ret[1] = Math.sin(alpha) * d;

        return ret;
    }

    /**
     * convert position (robot cord system) to position (field cord system)
     * @param dx x position (robot cord system)
     * @param dz z position (robot cord system)
     * @return   an Array with x and z position (field cord system)
     */
    protected double[] convert_rel2pos(double dx, double dz) {
        double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dz,2));
        double[] ret = {0,0};
        if (d == 0) {
            return ret;
        }

        double alpha = Math.asin(dz/d);
        if (dx < 0) {
            alpha = Math.PI - alpha;
        }
        alpha += Math.toRadians(rotation_y);

        ret[0] = Math.cos(alpha)*d;
        ret[1] = Math.sin(alpha)*d;

        return ret;
    }

    /**
     * drive a specified distance relative to the robot current position with a specified accuracy
     * @param dx    distance in x direction
     * @param dz    distance in z direction
     * @param speed speed factor
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
     * drive to a coordinate (field cord system) with a specified accuracy
     * @param x     x coordinate
     * @param z     z coordinate
     * @param speed speed factor
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
     * set speeds
     * @param vx    speed in x direction (robot cord system)
     * @param vz    speed in z direction (robot cord system)
     * @param wy    rotation speed of the robot
     * @param speed speed factor
     */
    public void drive_setSpeed(double vx, double vz, double wy, double speed) {
        this.vx = vx;
        this.vz = vz;
        this.wy = wy;
        this.drive_speed = speed;
        this.drive = false;
    }

    /**
     * set target rotation
     * @param target target rotation of the robot (field cord system)
     */
    public void set_targetRotation(double target) {
        if (target < -180) {
            target = 180 -(target % 180);
        } else if (target > 180) {
            target = -180 + (target % 180);
        }
        target_rotation_y = target;
    }

    /**
     * set motor speeds
     * @param vx    speed in x direction (robot cord system)
     * @param vz    speed in z direction (robot cord system)
     * @param wy    rotation speed of the robot
     * @param speed speed factor
     */
    protected void drive_setMotors(double vx, double vz, double wy, double speed) {
        // make sure the speeds are between -1 and 1
        double vmax = Math.max(Math.abs(vx), Math.abs(vz));
        if (vmax > 0) {
            if (Math.abs(speed) > 1) {
                speed /= Math.abs(speed);
            }
            vx = (vx/vmax) *speed;
            vz = (vz/vmax) *speed;
        }
        // wy *= speed;

        // get wheel speeds
        wheelSpeeds = calculateWheelSpeeds(vx, vz, wy);

        // set motor power
        robot.motor_front_left.setPower(wheelSpeeds[0]);
        robot.motor_front_right.setPower(wheelSpeeds[1]);
        robot.motor_rear_left.setPower(wheelSpeeds[2]);
        robot.motor_rear_right.setPower(wheelSpeeds[3]);
    }

    /**
     * stop controlled drive
     */
    public void drive_stop() {
        // overwrite vx and vz
        vx = 0;
        vz = 0;
        wy = 0;
        
        // Stop all motion;
        drive_setMotors(0, 0, 0, 0);

        // set drive flag
        drive = false;
    }

    /**
     * test if the driving target was reached
     * @return a Boolean indicating if the target was reached
     */
    protected boolean target_reached() {
        double d = Math.sqrt(Math.pow(target_position_x - position_x, 2) + Math.pow(target_position_z - position_z, 2));
        if (d <= Math.abs(drive_acc)) {
            // stop driving
            drive_stop();
            target_reached = true;
        }
        return target_reached;
    }

    /**
     * update position based on motor movement since the last step
     */
    protected void stepPos() {
        // get delta steps
        double delta_s1 = last_steps_fl - robot.motor_front_left.getCurrentPosition();
        double delta_s2 = last_steps_fr - robot.motor_front_right.getCurrentPosition();
        double delta_s3 = last_steps_rl - robot.motor_rear_left.getCurrentPosition();
        double delta_s4 = last_steps_rr - robot.motor_rear_right.getCurrentPosition();

        // set new last steps for next calculations
        last_steps_fl -= delta_s1;
        last_steps_fr -= delta_s2;
        last_steps_rl -= delta_s3;
        last_steps_rr -= delta_s4;

        // remove steps from gyro correction
        delta_s1 -= gyro_correction_steps[0];
        delta_s2 -= gyro_correction_steps[1];
        delta_s3 -= gyro_correction_steps[2];
        delta_s4 -= gyro_correction_steps[3];

        // reset gyro correction steps
        gyro_correction_steps[0] = 0;
        gyro_correction_steps[1] = 0;
        gyro_correction_steps[2] = 0;
        gyro_correction_steps[3] = 0;

        // calculate the distance
        double dx = (
                (R_D_FOUR*delta_s1*TWOPI_D_CPERMREV) +
                (R_D_FOUR*delta_s2*TWOPI_D_CPERMREV) +
                (R_D_FOUR*delta_s3*TWOPI_D_CPERMREV) +
                (R_D_FOUR*delta_s4*TWOPI_D_CPERMREV));
        double dz = (
                (R_D_FOUR*delta_s1*TWOPI_D_CPERMREV) -
                (R_D_FOUR*delta_s2*TWOPI_D_CPERMREV) -
                (R_D_FOUR*delta_s3*TWOPI_D_CPERMREV) +
                (R_D_FOUR*delta_s4*TWOPI_D_CPERMREV));

        // set new position
        double[] dp = convert_rel2pos(dx,dz);
        this.position_x -= dp[0];
        this.position_z -= dp[1];
    }

    /**
     * update motor speeds
     */
    protected void stepDrive() {
        if (drive && !target_reached()) {
            // gyro correction
            stepGyro();

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

    /**
     * update rotation speeds
     */
    protected void stepGyro() {
        // get rotation speed
        rotation_pi_controller.step(target_rotation_y-rotation_y);
        wy = rotation_pi_controller.out;

        gyro_correction_steps = calculateWheelSpeeds(0,0,wy);
    }

    /**
     * update robot rotation
     */
    protected void stepRotation() {
        // get rotation based on the start rotation
        rotation_y = gyro_start_rotation - gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + start_rotation_y;

        if (rotation_y < -180) {
            rotation_y = 180 -(rotation_y % 180);
        } else if (rotation_y > 180) {
            rotation_y = -180 + (rotation_y % 180);
        }

        rotation_y *= -1;
    }

    /**
     * step methode which executes step methodes
     */
    public void step() {
        stepRotation();
        stepDrive();
        stepPos();

    }
}
