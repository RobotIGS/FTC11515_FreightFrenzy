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

    public void driveToCarousel() {
        // TODO
    }

    public void rotateCarousel() {
        // TODO
    }

    public void parkInWarehouse() {
        omniWheel.setMotors(0.2,0,0);
        while (!ColorTools.isWhite(robot.colorSensor_down) && opModeIsActive()){}
        omniWheel.setMotors(0, 0, 0);

        controlledDrive.drive(20,0,0.15);
    }

    public int driveToCake(){
        int level = 1;

        // TODO: Drive forward with encoders

        if(robot.distanceSensor_front_mid.getDistance(DistanceUnit.CM)<=10) { //THIS VALUE NEEDS TO BE TESTED!! (Paul.U)
            level = 2;
        }

        // TODO: Drive sidewards with encoders

        if(robot.distanceSensor_front_mid.getDistance(DistanceUnit.CM)<=10) { //THIS VALUE NEEDS TO BE TESTED!! (Paul.U)
            level = 3;
        }
        return level;
    }

    public void driveToShippingHub() {
        // TODO
    }

    public void placeElementAtBottom() {
        // TODO
    }

    public void driveToWall() {
        // TODO 
    }

    public void placeElementAtPosition(BarcodeEnum barcodePosition) {
        // TODO
    }

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
}
