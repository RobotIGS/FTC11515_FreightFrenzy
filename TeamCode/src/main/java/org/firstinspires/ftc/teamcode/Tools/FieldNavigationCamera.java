package org.firstinspires.ftc.teamcode.Tools;

/*
// I think I can delete those, but who knows
import java.lang.reflect.Parameter;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
*/

import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import com.sun.tools.javac.util.Position;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


public class FieldNavigationCamera extends FieldNavigation {
    // true -> camera position is used to fix errors in position
    private boolean use_cam = true;

    private static final String VUFORIA_KEY = " --- YOUR NEW VUFORIA KEY GOES HERE  --- ";

    // Class Members
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables image_targets;
    private List<VuforiaTrackable> image_targets_array;
    private WebcamName camera;

    private boolean targetVisible       = false;

    // camera placement
    private OpenGLMatrix cameraLocationOnRobot;

    /**
     * one class to rule them all, (for the navigation of the robot) (can use camera to fix errors)
     * @param robot  BaseHardwareMap object
     * @param gyro   GyroHardwareMap object
     * @param camera WebcamName object
     * @param x      x start location
     * @param z      z start location
     * @param ry     start y rotation
     * @param cam_x  camera x position on robot
     * @param cam_y  camera y position on robot
     * @param cam_z  camera z position on robot
     * @param cam_rx camera x rotatoin on robot
     * @param cam_ry camera y rotation on robot
     * @param cam_rz camera z rotation on robot
     */
    public FieldNavigationCamera(BaseHardwareMap robot, GyroHardwareMap gyro, WebcamName camera,
        double x, double z, double ry,
        double cam_x, double cam_y, double cam_z,
        double cam_rx, double cam_ry, double cam_rz) {
        // hardware
        this.robot = robot;
        this.gyro = gyro;
        this.camera = camera;

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

        // camera location
        cameraLocationOnRobot = OpenGLMatrix
                .translation(cam_x, cam_y, cam_z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, cam_rx, cam_rz, cam_ry));

        // set vuforia tracking parameters (config)
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = camera;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        image_targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        image_targets_array = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // name and locate each trackable object
        //             id  name                  x y z  rx ry rz
        identifyTarget(0, "Blue Storage",        0,0,0, 0, 0, 0);
        identifyTarget(1, "Blue Alliance Wall",  0,0,0, 0, 0, 0);
        identifyTarget(2, "Red Storage",         0,0,0, 0, 0, 0);
        identifyTarget(3, "Red Alliance Wall",   0,0,0, 0, 0, 0);

        // set camera location
        updateCameraPlacement();
    }

    /**
     * set camer location on robot
     * @param x  x position
     * @param y  y position
     * @param z  z position
     * @param rx x rotation
     * @param ry y rotation
     * @param rz z rotation
     */
    public void setCameraPositionOnRobot(double x, double y, double z, double rx, double ry, double rz) {
        // TODO
        // set camera position

        // update position
        updateCameraPlacement();
    }

    /**
     * update camera position relative to the robot
     */
    private updateCameraPlacement() {
        /*  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : image_targets_array) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
    }

    /**
     * enable or disable the usage of vuforia to fix errors in the positioning
     * @param use_cam enables the use of vuforia
     */
    public void enable_cam(boolean use_cam) {
        this.use_cam = use_cam;
    }

    private void stepCam() {
        targetVisible = false;
        for (VuforiaTrackable trackable : image_targets_array) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    // update robot position
                    VectorF translation = robotLocationTransform.getTranslation();
                    position_x = translation.get(0);
                    position_z = translation.get(2);
                }
                break;
            }
        }
    }


    /**
     * go through every step methode
     */
    @Override
    public void step() {
        stepGyro();
        stepDrive();
        stepPos();
        if (use_cam) {
            stepCam();
        }
    }
}
