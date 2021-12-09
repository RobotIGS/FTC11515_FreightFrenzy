package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.BarcodeEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;
import org.firstinspires.ftc.teamcode.Tools.ColorTools;
import org.firstinspires.ftc.teamcode.Tools.ControlledDrive;
import org.firstinspires.ftc.teamcode.Tools.OmniWheel;

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

    public BarcodeEnum detectBarcodePosition() {
        boolean barcodePositionMid = false;
        boolean barcodePositionRight = false;
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
                    barcodePositionRight = true;
                    break;
                }
                lastBarcodeCheck = isBarcodeStoneNear();
            }

            controlledDrive.stop();
        }

        if (barcodePositionMid) return BarcodeEnum.Mid;
        else if (barcodePositionRight) return BarcodeEnum.Right;
        else return BarcodeEnum.Left;
    }

    public void driveToCarousel() {
        omniWheel.setMotors(-0.15,0,0);
        while (!(robot.distanceSensor_carousel.getDistance(DistanceUnit.CM) < 10) && opModeIsActive()){}
        omniWheel.setMotors(0, 0, 0);
    }

    public void rotateCarousel() {
        int direction = 1;

        if (getAllianceColor() == ColorEnum.Blue) {
            direction = -1;
        }

        robot.motor_carousel.setPower(0.5 * direction);

        long startTime = (new Date()).getTime();
        while (opModeIsActive()) {
            Date now = new Date();
            telemetry.addData("1", startTime + 1000);
            telemetry.addData("2", now.getTime());
            telemetry.update();
            if (startTime + 1000 < now.getTime()) {
                break;
            }
        } // wait a second

        robot.motor_carousel.setPower(0);
    }

    public void driveToShippingHub() {
        // TODO
    }

    public void placeElementAtBottom() {
        placeElementAtPosition(BarcodeEnum.Right);
    }

    public void placeElementAtPosition(BarcodeEnum barcodePosition) {
        // TODO
    }

    public void driveToWall() {
        // TODO
    }

    public void parkInWarehouse() {
        omniWheel.setMotors(0.2,0,0);
        while (!ColorTools.isWhite(robot.colorSensor_down) && opModeIsActive()){}
        omniWheel.setMotors(0, 0, 0);

        controlledDrive.drive(20,0,0.15);
    }
}
