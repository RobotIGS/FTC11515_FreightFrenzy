package org.firstinspires.ftc.teamcode.Tools;

/*
// I think I can delete those, but who knows
import java.lang.reflect.Parameter;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
*/

// import org.firstinspires.ftc.teamcode.Tools.FieldNavigation;
import org.firstinspires.ftc.teamcode.HardwareMaps.BaseHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareMaps.GyroHardwareMap;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


public class FieldNavigationCamera extends FieldNavigation {
    // true -> camera position is used to fix errors in position
    private boolean use_cam = true;

    private static final String VUFORIA_KEY = "AfJ0TyL/////AAABmd78ofn/RkMRi5drULeQkx9J7iXzq0RVLEWyuyfXRDN3IoVgx67f+ACtVorRwa96Jnk49/2xCVBKEeei3RC9zoBnb3genq9MMD6y4kXKbyQIuFN7xispFh7+SfEtm59sNU3R5GJfTAOym68R1IU+4rgY+G4ISATIz3Y9qLBzScQDRqILmn/yGBmC2i+lw8aDepPuAND4he/bkN2ONnp5U8XBAlrZmuPWzRb63RBo5RBdWi19D3h0FOK7KgUV0sgThso9FPVRhDKqB8swS9AqcGIbMo3lqgRA/w7ON5hnRJj6RG+GV+CNDcObyiwMCtEhYaisfR6pNg1NrUTU2Cxgv6291o8fgThPYT9DNKdjz3Um";

    // Class Members
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables image_targets;
    private List<VuforiaTrackable> image_targets_array;
    private WebcamName camera;

    private VuforiaLocalizer.Parameters parameters;

    public boolean targetVisible       = false;

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
        double cam_rx, double cam_ry, double cam_rz,
        double PI_d, double PI_i) {
        super(robot, gyro, x, z, ry, PI_d, PI_d);

        // camera location
        cameraLocationOnRobot = OpenGLMatrix
                .translation((float) cam_x, (float) cam_y, (float) cam_z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, (float) cam_rx, (float) cam_rz, (float) cam_ry));

        // set vuforia tracking parameters (config)
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = camera;
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        image_targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        image_targets_array = new ArrayList<VuforiaTrackable>();
        image_targets_array.addAll(image_targets);

        // name and locate each trackable object
        //             id  name                  x y z  rx ry rz
        identifyTarget(3, "Blue Storage",        -179.0f,15.24f,-91.0f, 0, 90, 0);
        identifyTarget(0, "Blue Alliance Wall",  -88.5f,15.24f,180.0f, 0, 180, 0);
        identifyTarget(1, "Red Storage",         179.0f,15.24f,-90.0f, 0, -90, 0);
        identifyTarget(2, "Red Alliance Wall",   88.0f,15.24f,179.0f, 0, 180, 0);

        // set camera location
        updateCameraPlacement();

        // activate targets
        image_targets.activate();
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
    private void updateCameraPlacement() {
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
        if (use_cam) {
            image_targets.activate();
        } else {
            image_targets.deactivate();
        }
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
        stepRotation();
        stepDrive();
        stepPos();
        if (use_cam) {
            stepCam();
        }
    }
    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = image_targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
