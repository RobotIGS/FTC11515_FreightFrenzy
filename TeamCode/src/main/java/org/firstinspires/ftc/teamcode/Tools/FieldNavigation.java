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

    /*
     * live data
    */

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

    // rotation of the robot
    public double rotation_y;

    private double wheelSpeeds[] = {0,0,0,0};

    /*
     * target data
    */

    // target position for navigation
    private double target_position_x;
    private double target_position_z;
    private double target_step_position_x;
    private double target_step_position_z;
    private double drive_acc;

    // target rotation of the robot
    private double target_rotation_y;
    private double start_rotation_y;
    private double gyro_start_rotation;

    /*
     * drive constants
    */

    private static final double DRIVE_STEP_ACC = 4;
    private static final double DRIVE_STEP_LENGTH = 10; // max cm

    /*
     * drive flags and vars
    */

    private boolean drive = false;
    private boolean target_reached = false;
    private double drive_target_acc;
    private double drive_speed;

    /* constructor */

    /**
     * one class to rule them all, (for the navigation of the robot)
     * @param robot BaseHardwareMap object
     * @param x     x start location
     * @param z     z start location
     * @param ry    start y rotation
     */
    public FieldNavigation(BaseHardwareMap robot, double x, double z, double ry) {
        // hardware
        this.robot = robot;
        gyro = new GyroHardwareMap(hardwareMap);
        hwgy.init(hardwareMap);

        // set start rotation (gyro)
        gyro_start_rotation = gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle();

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
    private double[] calculateWheelSpeeds(double vx, double vz, double wy) {
        wheelSpeeds[0] = vx - vz - wy;
        wheelSpeeds[1] = vx + vz + wy;
        wheelSpeeds[2] = vx + vz - wy;
        wheelSpeeds[3] = vx - vz + wy;
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
     * set the step targets of the motors and start driving to them
     * @param dx distance in x direction in cm relative to the robot
     * @param dz distance in z direction in cm relative to the robot
     */
    private void setWheelTargets(double dx, double dz, double speed) {
        // calculate max distance 
        double d_max = Math.max(Math.abs(dx), Math.abs(dz));

        // normalize (vals from -1 to 1) -> translate to relative speeds
        dx /= d_max;
        dz /= d_max;

        // get wheelSpeeds
        calculateWheelSpeeds(dx,dz,0);

        // get targets
        double wheelTargets[] = calculateWheelTargets(wheelSpeeds, d_max);

        // set targets
        robot.motor_front_left.setTargetPosition((int) wheelTargets[0]);
        robot.motor_front_right.setTargetPosition((int) wheelTargets[1]);
        robot.motor_rear_left.setTargetPosition((int) wheelTargets[2]);
        robot.motor_rear_right.setTargetPosition((int) wheelTargets[3]);

        // Turn On RUN_TO_POSITION mode
        robot.motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_rear_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // check speed
        if (speed > 1 || speed < -1) {
            speed /= Math.abs(speed);
        }

        // set motor power
        robot.motor_front_left.setPower(wheelSpeeds[0] * speed);
        robot.motor_front_right.setPower(wheelSpeeds[1] * speed);
        robot.motor_rear_left.setPower(wheelSpeeds[2] * speed);
        robot.motor_rear_right.setPower(wheelSpeeds[3] * speed);
    }

    /**
     * drive a specified distance relative to the robot current position and rotation with a specified accuracy
     * @param dx    distance in x direction
     * @param dz    distance in z direction
     * @param speed speed
     * @param acc   accuracy in cm (per motor)
     */
    public void drive_rel(double dx, double dz, double speed, double acc) {
        setWheelTargets(dx,dz,speed);

        // set drive flag and target acc
        drive = true;
        drive_target_acc = acc;
        target_reached = false;
        target_step_reached = true;
    }

    /**
     * drive a specified distance relative to the robot current position and rotation
     * @param dx    distance in x direction
     * @param dz    distance in z direction
     * @param speed speed
     */
    public void drive_rel(double dx, double dz, double speed) {
        drive_rel(dx,dz,speed,0);
    }

    /**
     * set target position
     * @param x     x coordinate
     * @param z     z coordinate
     * @param speed speed
     * @param acc   accuracy in cm (per motor)
     */
    public void set_target_position(double x, double z, double speed, double acc) {
        // TODO
        // set target position
        drive_target_acc = acc;
        drive_speed = speed;
        target_step_reached = true;
        //drive_rel
    }

    /**
     * drive to a coordinate on the field with a specified accuracy
     * @param x     x coordinate
     * @param z     z coordinate
     * @param speed speed
     * @param acc   accuracy in cm (per motor)
     */
    public void drive_to_pos(double x, double z, double speed, double acc) {
        // TODO calc rel to robot -> drive_rel (USING STEP TARGETS!!!!)
    }

    /**
     * drive to a coordinate on the field
     * @param x     x coordinate
     * @param z     z coordinate
     * @param speed speed
     */
    public void drive_to_pos(double x, double z, double speed) {
        drive_to_pos(x,z,speed,0);
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

        // set drive flag
        drive = false;
    }

    /**
     * test if the driving target was reached
     * @return a Boolean if the target was reached
     */
    public boolean drive_finished() {
        return target_reached;
    }

    /**
     * navigation based on motor movement
     */
    private void stepPos() {
        // get delta steps
        double delta_s1 = last_steps_fl - robot.motor_front_left.getCurrentPosition();
        double delta_s2 = last_steps_fr - robot.motor_front_right.getCurrentPosition();
        double delta_s3 = last_steps_rl - robot.motor_rear_left.getCurrentPosition();
        double delta_s4 = last_steps_rr - robot.motor_rear_right.getCurrentPosition();

        // calculate the distance
        double dx = (delta_s1 + delta_s2 + delta_s3 + delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);
        double dz = (delta_s1 - delta_s2 - delta_s3 + delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);

        // set new position
        double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dz,2));
        if (d == 0) {
            return;
        }
        double alpha = Math.asin(dx/d) - Math.toRadians(rotation_y);

        this.position_x -= Math.sin(alpha)*d;
        this.position_z -= Math.cos(alpha)*d;

        // set new last steps for next calculations
        last_steps_fl -= delta_s1;
        last_steps_fr -= delta_s2;
        last_steps_rl -= delta_s3;
        last_steps_rr -= delta_s4;
    }

    /**
     * controlled drive stuff
     */
    private void stepDrive() {
        if (drive) {
            if (target_step_reached) {
                // test if target is reached
                double distance_to_target = Math.sqrt(Math.pow(position_x - target_position_x, 2) + Math.pow(position_z - target_position_z, 2));

                // test if real target is reached
                if (distance_to_target < drive_target_acc) {
                    drive_stop();
                } else {
                    // step target
                    if (distance_to_target > DRIVE_STEP_LENGTH) {
                        double dr = Math.sqrt(Math.pow(target_position_x - position_x, 2) + Math.pow(target_position_z - position_z, 2)) / DRIVE_STEP_LENGTH;
                        target_step_position_x = position_x + (target_position_x - position_x) / dr;
                        target_step_position_z = position_z + (target_position_z - position_z) / dr;
                        drive_acc = DRIVE_STEP_ACC;
                    }

                    // real target
                    else {
                        target_step_position_x = target_position_x;
                        target_step_position_z = target_position_z;
                        drive_acc = drive_target_acc;
                    }

                    // test if (step) target is reached
                    if (Math.sqrt(Math.pow(position_x - target_step_position_x, 2) + Math.pow(position_z - target_step_position_z, 2)) < drive_acc) {
                        target_step_reached = true;
                    }
                }
            }
        }
    }

    private void stepGyro() {
        // get rotation based on the start rotation
        rotation_y = gyro_start_rotation - gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle() + start_rotation_y;

        if (Math.abs(rotation_y) > 180) {
            // TODO fix this syntax error
            rotation_y += 180 *(rotation_y % 180) * (rotation_y < 0);
        }
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
