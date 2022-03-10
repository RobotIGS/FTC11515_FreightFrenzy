package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.*;

import java.util.Date;

public abstract class BaseAutonomous extends LinearOpMode {
    BaseHardwareMap robot;
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

        controlledDrive.drive(5,0,0.15);

        controlledDrive.start(-15,0,0.15);
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
        omniWheel.setMotors(-0.25,0,0);
        while (!(robot.distanceSensor_carousel.getDistance(DistanceUnit.CM) < 15) && opModeIsActive()){}
        omniWheel.setMotors(0, 0, 0);

        omniWheel.setMotors(-0.15,0,0.15);
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
        int dirMul = (getAllianceColor() == ColorEnum.Blue) ? 1 : -1;

        // just orient at wall
        controlledDrive.drive(25,0, 0.3);

        controlledDrive.drive(25, 130*dirMul, 0.18);
    }

    public void placeElementAtPosition(PositionEnum position) {
        int encoderAmount = 0;
        switch (position) {
            case Bottom:
                encoderAmount = -600;
                break;
            case Middle:
                encoderAmount = -1000;
                break;
            case Top:
                encoderAmount = -1500;
                break;
        }
        int startPos = robot.motor_lift.getCurrentPosition();
        robot.motor_lift.setTargetPosition(startPos + encoderAmount);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.5);

        while (robot.motor_lift.isBusy() && opModeIsActive()) {}

        robot.motor_shovel.setPower(-0.75);
        long startTime = (new Date()).getTime();
        while (opModeIsActive()) {
            Date now = new Date();
            if (startTime + 500 < now.getTime()) {
                break;
            }
        } // wait half a second
        robot.motor_shovel.setPower(0);

        robot.motor_lift.setTargetPosition(startPos);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motor_lift.setPower(0.5);
        while (robot.motor_lift.isBusy() && opModeIsActive()) {}
        robot.motor_lift.setPower(0);
        robot.motor_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveToWall() {
        int dirMul = (getAllianceColor() == ColorEnum.Blue) ? 1 : -1;

        controlledDrive.drive(70, -140*dirMul, 0.25);
    }

    public void parkInWarehouse() {
        omniWheel.setMotors(0.35,0,0);
        while (!ColorTools.isWhite(robot.colorSensor_down) && opModeIsActive()){}
        omniWheel.setMotors(0, 0, 0);

        controlledDrive.drive(20,0,0.3);
    }
}
