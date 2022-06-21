package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;

public class FieldNavigation {
    // robot object
    BaseHardwareMap robot;

    // statics
    static final double COUNTS_PER_MOTOR_REV = 751.8; // 223rpm
    //static final double COUNTS_PER_MOTOR_REV = 384.5; // 435rpm
    static final double DRIVE_GEAR_REDUCTION = 1.0;      // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CMS = 10.0;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CMS * Math.PI);

    // robot stuff (constants)
    static final double lx = 1;
    static final double ly = 1;

    // last steps of the motors
    private double last_steps_fl;
    private double last_steps_fr;
    private double last_steps_rl;
    private double last_steps_rr;

    // position of the robot
    public double position_x;
    public double position_y;

    // controlled drive
    private boolean drive;
    private double drive_acc;
    private boolean target_reached;

    /* constructor */

    public FieldNavigation(BaseHardwareMap robot, double x, double y) {
        this.robot = robot;
        last_steps_fl = robot.motor_front_left.getCurrentPosition();
        last_steps_fr = robot.motor_front_right.getCurrentPosition();
        last_steps_rl = robot.motor_rear_left.getCurrentPosition();
        last_steps_rr = robot.motor_rear_right.getCurrentPosition();

        position_x = x;
        position_y = y;

        drive = false;
        target_reached = false;
    }

    /**
     * calculate the speed of the motors out of the speed from the robot
     * @param vx spped in x direction
     * @param vy spped in y direction
     * @param wz rotation speed (0)
     * @return an array with motor speeds (4x1) (front left, front right, rear left, rear right)
     */
    private double[] calculateWheelSpeeds(double vx, double vy, double wz) {
        double[] wheelSpeeds = {
            vx + vy - wz,
            vx - vy + wz,
            vx - vy - wz,
            vx + vy + wz
        };

        return wheelSpeeds;
    }

    /**
     * calculate wheel step targets
     * @param wheelSpeeds an array with the speeds of the motors (return from calculateWheelSpeeds)
     * @param maxDistance the absolute maximum distance of x and y in cm
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
     * @param dy distance in y direction in cm relative to the robot
     */
    private void setWheelTargets(double dx, double dy, double speed) {
        // calculate max distance 
        double d_max = Math.max(Math.abs(dx), Math.abs(dy));

        // normalize (vals from -1 to 1) -> translate to relative speeds
        dx /= d_max;
        dy /= d_max;

        // get wheelSpeeds
        double[] wheelSpeeds = calculateWheelSpeeds(dx,dy,0);

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
     * @param dy    distance in y direction
     * @param speed speed
     * @param acc   accuracy in cm (per motor)
     */
    public void drive_rel(double dx, double dy, double speed, double acc) {
        setWheelTargets(dx,dy,speed);

        // set drive flag and target acc
        drive = true;
        drive_acc = acc;
        target_reached = false;
    }

    /**
     * drive a specified distance relative to the robot current position and rotation
     * @param dx    distance in x direction
     * @param dy    distance in y direction
     * @param speed speed
     */
    public void drive_rel(double dx, double dy, double speed) {
        drive_rel(dx,dy,speed,0);
    }

    /**
     * drive to a coordinate on the field with a specified accuracy
     * @param x     x coordinate
     * @param y     y coordinate
     * @param speed speed
     * @param acc   accuracy in cm (per motor)
     */
    public void drive_to_pos(double x, double y, double speed, double acc) {
        // TODO
        // calc rel dis from pos to (x|y) remember to use the robot rotation in the calculation
        // set targets using drive_rel
    }

    /**
     * drive to a coordinate on the field
     * @param x     x coordinate
     * @param y     y coordinate
     * @param speed speed
     */
    public void drive_to_pos(double x, double y, double speed) {
        drive_to_pos(x,y,speed,0);
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
     * test if the controlled drive has finished
     */
    private void drive_test() {
        // if acc == 0cm -> just test if no motor is busy
        if (drive_acc == 0) {
            if (
                    !robot.motor_front_left.isBusy() &&
                    !robot.motor_front_right.isBusy() &&
                    !robot.motor_rear_left.isBusy() &&
                    !robot.motor_rear_right.isBusy()
               ) {
                drive_stop();
                target_reached = true;
           }
        }

        // if acc > 0cm
        else if (
                Math.abs(robot.motor_front_left.getTargetPosition() - robot.motor_front_left.getCurrentPosition()) * COUNTS_PER_CM <= drive_acc &&
                Math.abs(robot.motor_front_right.getTargetPosition() - robot.motor_front_right.getCurrentPosition()) * COUNTS_PER_CM <= drive_acc &&
                Math.abs(robot.motor_rear_left.getTargetPosition() - robot.motor_rear_left.getCurrentPosition()) * COUNTS_PER_CM <= drive_acc &&
                Math.abs(robot.motor_rear_right.getTargetPosition() - robot.motor_rear_right.getCurrentPosition()) * COUNTS_PER_CM <= drive_acc
           ) {
            drive_stop();
            target_reached = true;
        }
    }

    /**
     * test if the driving target was reached
     * @return a Boolean if the target was reached
     */
    public boolean drive_finished() {
        return target_reached;
    }

    /**
     * camera navigation stuff
     */
    private void stepCam() {}

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
        double dx = ( delta_s1 + delta_s2 + delta_s3 + delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);
        double dy = (delta_s1 - delta_s2 - delta_s3 + delta_s4) * ((2*Math.PI) / COUNTS_PER_MOTOR_REV);

        // set new position
        position_x -= dx;
        position_y -= dy;

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
            drive_test();
        }
    }

    /**
     * go through every step methode
     */
    public void step() {
        stepDrive();
        stepPos();
        stepCam();
    }
}
