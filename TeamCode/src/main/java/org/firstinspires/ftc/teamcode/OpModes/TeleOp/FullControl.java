package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class FullControl extends BaseTeleOp {
    boolean intake;
    int liftStartPos;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        intake = false;
        liftStartPos = robot.motor_lift.getCurrentPosition(); // TODO: get val from auton..
    }

    @Override
    public void loop() {
        double diff = liftStartPos - robot.motor_lift.getCurrentPosition();
        double vflift = 0;
        double vlift = 0;
        omniWheel.setMotors(
                -gamepad1.left_stick_y * 0.7,
                gamepad1.left_stick_x * 0.7,
                gamepad1.right_stick_x * 0.2
        );

        telemetry.addData("dis", robot.distanceSensor_right.getDistance(DistanceUnit.CM));

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
        /*
        if (robot.motor_lift.getCurrentPosition() - liftStartPos > 0) {
            if (gamepad1.right_stick_y > 0) {
                robot.motor_lift.setPower(-gamepad1.right_stick_y * 0.4);
            }
        }
        else {
            robot.motor_lift.setPower(-0.1);
        }
        if (robot.motor_lift.getCurrentPosition() - liftStartPos < 2700) {
            if (gamepad1.right_stick_y < 0) {
                robot.motor_lift.setPower(-gamepad1.right_stick_y * 0.4);
            }
        }
        else {
            robot.motor_lift.setPower(0.1);
        }

        if (gamepad1.right_stick_y > 0) {
            robot.motor_lift.setPower(-gamepad1.right_stick_y * 0.4);
        } else if (gamepad1.right_stick_y < 0) {
            robot.motor_lift.setPower(-gamepad1.right_stick_y * 0.4);
        } else {
            robot.motor_lift.setPower(0);
        }
        */

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
        telemetry.update();
    }
}
