package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMaps.MotorHardwareMap;

@TeleOp
public class MotorTest extends BaseTeleOp {
    @Override
    public void initialize() {
        robot = new MotorHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {
        robot.motor_front_left.setPower(0.6);
        if (gamepad1.a) {
            robot.motor_front_left.setPower(0.6);
        } else if (gamepad1.b){
            robot.motor_front_left.setPower(0);
        }
        if (gamepad1.x) {
            robot.servo_Lift.setPower(1);
        } else if (gamepad1.y){
            robot.servo_Lift.setPower(-1);
            robot.servo_Lift.set 
        } else {
            robot.servo_Lift.setPower(0);
        }
    }
}
