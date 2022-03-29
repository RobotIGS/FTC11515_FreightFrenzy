package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
        return (getBarcodeDistanceSensor().getDistance(DistanceUnit.CM) < 100);
    }

    public PositionEnum detectBarcodePosition() {
        boolean barcodePositionMid = false;
        boolean barcodePositionFront = false;
        boolean lastBarcodeCheck = false;

        controlledDrive.drive(5, 0, 0.15);

        controlledDrive.start(-15, 0, 0.15);
        while (opModeIsActive() && (robot.motor_front_left.isBusy() ||
                robot.motor_front_right.isBusy() || robot.motor_rear_left.isBusy() ||
                robot.motor_rear_right.isBusy())) {
            if (isBarcodeStoneNear() && lastBarcodeCheck) {
                barcodePositionMid = true;
                break;
            }
            lastBarcodeCheck = isBarcodeStoneNear();
        }
        controlledDrive.stop();

        lastBarcodeCheck = false;

        if (!barcodePositionMid) {
            controlledDrive.start(-15, 0, 0.15);
            while (opModeIsActive() && (robot.motor_front_left.isBusy() ||
                    robot.motor_front_right.isBusy() || robot.motor_rear_left.isBusy() ||
                    robot.motor_rear_right.isBusy())) {
                if (isBarcodeStoneNear() && lastBarcodeCheck) {
                    barcodePositionFront = true;
                    break;
                }
                lastBarcodeCheck = isBarcodeStoneNear();
            }

            controlledDrive.stop();
        }

        BarcodeEnum barcodeState;

        if (barcodePositionMid) barcodeState = BarcodeEnum.Mid;
        else if (barcodePositionFront) barcodeState = BarcodeEnum.Front;
        else barcodeState = BarcodeEnum.Back;

        boolean isBlue = getAllianceColor() == ColorEnum.Blue;
        switch (barcodeState) {
            case Back:
                if (isBlue) return PositionEnum.Top;
                return PositionEnum.Bottom;
            case Mid:
                return PositionEnum.Middle;
            case Front:
                if (isBlue) return PositionEnum.Bottom;
                return PositionEnum.Top;
        }

        return PositionEnum.Bottom;
    }

    public void driveToCarousel() {
        omniWheel.setMotors(-0.15, 0, 0); // changed 025 to 015
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
        omniWheel.setMotors(0, 0, 0.001);
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
        Orientation angles;

        // drive back
        controlledDrive.drive(125, 0, 0.2);

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

        // get a clear view (move the lift out of the way from the dis sensor)
        robot.motor_lift.setTargetPosition(liftStartPos + 700);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.2);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {
        }
        robot.motor_lift.setPower(0);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // forward to the ShippingHub
        controlledDrive.drive(35, 0, 0.2);
    }

    public void placeElementAtPosition(PositionEnum position) {
        int encoderAmount = 0;
        switch (position) {
            case Bottom:
                encoderAmount = 700;
                break;
            case Middle:
                encoderAmount = 1000;
                break;
            case Top:
                encoderAmount = 1500;
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
        controlledDrive.drive(-37, 0, 0.2);
    }

    public void parkInWarehouse() {
        int dirMul = (getAllianceColor() == ColorEnum.Blue) ? 1 : -1;

        omniWheel.setMotors(0, -0.2 * dirMul, 0);
        while (!ColorTools.isWhite(robot.colorSensor_down) && opModeIsActive()) {
        }
        omniWheel.setMotors(0, 0, 0);

        controlledDrive.drive(0, -20 * dirMul, 0.2);
    }
}
