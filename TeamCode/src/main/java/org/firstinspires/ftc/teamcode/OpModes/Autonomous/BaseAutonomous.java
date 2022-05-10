package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.BarcodeEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorTools;
import org.firstinspires.ftc.teamcode.Tools.ControlledDrive;
import org.firstinspires.ftc.teamcode.Tools.OmniWheel;
import org.firstinspires.ftc.teamcode.Tools.PositionEnum;

import java.util.Date;

public abstract class BaseAutonomous extends LinearOpMode {
    double gyroPosition;
    int liftStartPos;
    PositionEnum barcodePosition;
    BaseHardwareMap robot;
    GyroHardwareMap hwgy;
    OmniWheel omniWheel;
    ControlledDrive controlledDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        run();
    }

    void initialize() {
        robot = initializeHardwareMap();
        omniWheel = new OmniWheel(robot);
        controlledDrive = new ControlledDrive(robot, this);
        hwgy = new GyroHardwareMap(hardwareMap);
        hwgy.init(hardwareMap);
        gyroPosition = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        liftStartPos = robot.motor_lift.getCurrentPosition();
        barcodePosition = PositionEnum.unknown;
    }

    public abstract BaseHardwareMap initializeHardwareMap();

    public abstract ColorEnum getAllianceColor();

    public abstract void run();

    public DistanceSensor getBarcodeDistanceSensor() {
        if (getAllianceColor() == ColorEnum.Blue) {
            return robot.distanceSensor_right;
        } else {
            return robot.distanceSensor_left;
        }
    }

    boolean isBarcodeStoneNear() {
        double dis = getBarcodeDistanceSensor().getDistance(DistanceUnit.CM);
        telemetry.addData("dis", dis);
        telemetry.update();
        return (dis < 100 && dis > 45);
    }

    public void detectBarcodePosition() {
        int counter = 0;
        int n_counters = 5; // counters to detect the duck / cube / .. on the barcode

        // middle barcode mark
        controlledDrive.start(-18, 0, 0.2);
        while (opModeIsActive() && !controlledDrive.test_acc(100)) {
            if (isBarcodeStoneNear()) {
                counter ++;
            }
        }
        controlledDrive.stop();

        // check if the sensor found the duck / .. more than n_counters times
        if (counter > n_counters) {
            barcodePosition = PositionEnum.Middle;
        }

        // skip if the duck / cube / .. was already detected
        if (barcodePosition == PositionEnum.unknown) {
            counter = 0;

            // the barcode marker next to the wall
            controlledDrive.start(-18, 0, 0.2);
            while (opModeIsActive() && !controlledDrive.test_acc(100)) {
                if (isBarcodeStoneNear()) {
                    counter ++;
                }
            }
            controlledDrive.stop();

            // check if the sensor found the duck / .. more than n_counters times
            if (counter > n_counters) {
                barcodePosition = PositionEnum.Bottom;
            }
        }
        
        // flip the psitions if getAllianceColor is red
        if (getAllianceColor() == ColorEnum.Red && barcodePosition == PositionEnum.Bottom) {
            barcodePosition = PositionEnum.Top;
        }

        // handle missing barcode marker
        if (barcodePosition == PositionEnum.unknown) {
            if (getAllianceColor() == ColorEnum.Red) {
                barcodePosition = PositionEnum.Bottom;
            } else {
                barcodePosition = PositionEnum.Top;
            }
        }

        // DEBUG:
        telemetry.addData("pos", barcodePosition);
        telemetry.addData("counter", counter);
        telemetry.update();
    }

    public void driveToCarousel() {
        double dirMul=1;
        if (getAllianceColor() != ColorEnum.Blue) {
            dirMul = -1;
        }
        // detect barcode
        omniWheel.setMotors(-0.15, 0, 0); // changed 025 to 015 + -0.01 drive against the wall
        while (!(robot.distanceSensor_carousel.getDistance(DistanceUnit.CM) < 20) && opModeIsActive()) {
        } // 15 to 20
        omniWheel.setMotors(0, 0, 0);

        omniWheel.setMotors(-0.15, 0, 0.15);
        long startTime = (new Date()).getTime();
        while (opModeIsActive()) {
            Date now = new Date();
            if (startTime + 100 < now.getTime()) {
                break;
            }
        } // wait a moment
        omniWheel.setMotors(0, 0, 0.001*dirMul);
    }

    public void rotateCarousel() {
        int direction = 1;

        if (getAllianceColor() == ColorEnum.Blue) {
            direction = -1;
        }

        robot.motor_carousel.setPower(0.55 * direction);

        long startTime = (new Date()).getTime();
        while (opModeIsActive()) {
            Date now = new Date();
            if (startTime + 3000 < now.getTime()) {
                break;
            }
        } // wait a second

        robot.motor_carousel.setPower(0);
    }

    public void driveToShippingHub() {
        double angleError;
        double vr = 0;
        int encoderAmount = 0;
        Orientation angles;

        // drive back
        controlledDrive.drive_with_acc(125, 0, 0.2, 100);

        do {
            angles = hwgy.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleError = gyroPosition - angles.firstAngle;
            // rotate 90 deg
            if (getAllianceColor() == ColorEnum.Blue) {
                angleError -= 90;
            } else {
                angleError += 90;
            }
            // no deg > 180 or < -180
            if (angleError > 180) {
                angleError -= 180;
            } else if (angleError < -180) {
                angleError += 180;
            }
            // get rotation speed vr and rotate
            vr = angleError / 180 * 0.3;
            omniWheel.setMotors(0, 0, -vr);
        } while (!(angleError < 4 && angleError > -4) && opModeIsActive());
        // rotate until the error is > -5 and < 5
        omniWheel.setMotors(0, 0, 0);

        // start driving forward
        controlledDrive.start(35,0,0.2);

        // get a clear view (move the lift out of the way from the dis sensor)
        robot.motor_lift.setTargetPosition(liftStartPos + 800);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.2);
        while (robot.motor_lift.isBusy() && opModeIsActive() && !controlledDrive.test_acc(100)) {
            if (controlledDrive.test_acc(100)) {
                controlledDrive.stop();
            }
            if (!robot.motor_lift.isBusy()) {
                robot.motor_lift.setPower(0);
                robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        // to make shure, that the robot stop moving stuff
        robot.motor_lift.setPower(0);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controlledDrive.stop();
    }

    public void placeElementAtPosition(PositionEnum position) {
        int encoderAmount = 0;
        switch (position) {
            case Bottom:
                encoderAmount = 800;
                break;
            case Middle:
                encoderAmount = 1600;
                break;
            case Top:
                encoderAmount = 2600;
                break;
        }
        robot.motor_lift.setTargetPosition(liftStartPos + encoderAmount);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.5);

        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }

        robot.motor_shovel.setPower(-0.75);
        long startTime = (new Date()).getTime();
        while (opModeIsActive()) {
            Date now = new Date();
            if (startTime + 1000 < now.getTime()) {
                break;
            }
        } // wait half a second
        robot.motor_shovel.setPower(0);

        robot.motor_lift.setTargetPosition(liftStartPos + 700);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.5);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }
        robot.motor_lift.setPower(0);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveToWall() {
        controlledDrive.drive_with_acc(-37, 0, 0.2, 100);
    }

    public void parkInWarehouse() {
        int dirMul = (getAllianceColor() == ColorEnum.Blue) ? 1 : -1;

        omniWheel.setMotors(-0.06, -0.2 * dirMul, 0);
        while (!ColorTools.isWhite(robot.colorSensor_down) && opModeIsActive()) {
        }
        omniWheel.setMotors(0, 0, 0);

        controlledDrive.drive_with_acc(0, -30 * dirMul, 0.2, 100);
    }
}
