package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.FullHardwareMap;

@TeleOp
public class FullControl extends BaseTeleOp {
    boolean intake;
    int liftStartPos;

    @Override
    public void initialize() {
        robot = new FullHardwareMap(hardwareMap);
        intake = false;
        liftStartPos = robot.motor_lift.getCurrentPosition();
    }

    @Override
    public void loop() {
        omniWheel.setMotors(
                -gamepad1.left_stick_y * 0.7,
                gamepad1.left_stick_x * 0.7,
                gamepad1.right_stick_x * 0.2
        );

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

        telemetry.addData("lift", liftStartPos - robot.motor_lift.getCurrentPosition());
        telemetry.update();
    }
}
