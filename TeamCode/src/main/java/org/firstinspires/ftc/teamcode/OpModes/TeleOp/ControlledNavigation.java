package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class ControlledNavigation extends BaseTeleOp{
    private static final String VUFORIA_KEY =
            "AfJ0TyL/////AAABmd78ofn/RkMRi5drULeQkx9J7iXzq0RVLEWyuyfXRDN3IoVgx67f+ACtVorRwa96Jnk49/2xCVBKEeei3RC9zoBnb3genq9MMD6y4kXKbyQIuFN7xispFh7+SfEtm59sNU3R5GJfTAOym68R1IU+4rgY+G4ISATIz3Y9qLBzScQDRqILmn/yGBmC2i+lw8aDepPuAND4he/bkN2ONnp5U8XBAlrZmuPWzRb63RBo5RBdWi19D3h0FOK7KgUV0sgThso9FPVRhDKqB8swS9AqcGIbMo3lqgRA/w7ON5hnRJj6RG+GV+CNDcObyiwMCtEhYaisfR6pNg1NrUTU2Cxgv6291o8fgThPYT9DNKdjz3Um";

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;

    private List<VuforiaTrackable> allTrackables = null;

    private boolean intake;
    private int liftStartPos;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        intake = false;
        liftStartPos = robot.motor_lift.getCurrentPosition(); // TODO: get val from auton..

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.addWebcamCalibrationFile("xml/teamwebcamcalibrations.xml");

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = true;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Name and locate each trackable object (NOT REAL VALUES!!!!!)
        identifyTarget(0, "blue A wall", -88.5f, 15.24f,180.0f, 0, 180, 0);
        identifyTarget(3, "blue A storage", -179.0f, 15.24f, -91.0f, 0, 90, 0);
        identifyTarget(2, "red  A wall", 88.0f, 15.24f, 179.0f, 0, 180, 0);
        identifyTarget(1, "red  A storage", 179.0f, 15.24f, -90.0f, 0, -90, 0);

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 0, 0, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();
    }

    @Override public void loop() {
        double diff = liftStartPos - robot.motor_lift.getCurrentPosition();
        double vflift = 0;
        double vlift = 0;
        omniWheel.setMotors(
                -gamepad1.left_stick_y * 0.7,
                gamepad1.left_stick_x * 0.7,
                gamepad1.right_stick_x * 0.2
        );

        // make the lift slower at the end region of the allowed movement region
        if (diff > -300 || diff < -4100) {
            vflift = -0.3;
        } else {
            vflift = -0.5;
        }

        // 0 min; -340 max
        if (diff < -60 && gamepad1.right_stick_y > 0) {
            vlift = vflift*gamepad1.right_stick_y;
        } else if (diff > -3300 && gamepad1.right_stick_y < 0) {
            vlift = vflift*gamepad1.right_stick_y;
        } else if (diff > 0) {
            vlift = 0.1;
        } else if (diff < -3400) {
            vlift = -0.1;
        }
        robot.motor_lift.setPower(vlift);

        if (gamepad1.a){
            intake = !intake;
            robot.motor_shovel.setPower(0);
        }
        if (gamepad1.b){
            robot.motor_shovel.setPower(-0.5);
            intake = false;
        }
        else if (intake) {
            robot.motor_shovel.setPower(1);
        }
        else {
            robot.motor_shovel.setPower(0);
        }

        if (gamepad1.x) {
            robot.motor_carousel.setPower(0.5);
        }
        else if (gamepad1.y) {
            robot.motor_carousel.setPower(-0.5);
        } else {
            robot.motor_carousel.setPower(0);
        }

        telemetry.addData("lift", diff);

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (cm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex
     * @param targetName
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    @Override
    public void stop() {
        // Disable Tracking when we are done;
        targets.deactivate();
    }
}