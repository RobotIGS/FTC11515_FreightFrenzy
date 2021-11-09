package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.Tools.ColorEnum;
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
        // TODO:
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

    public void placeElementAtPosition() {
        // TODO
    }

    public void detectPositionBarcode(){
        // TODO
    }
}
