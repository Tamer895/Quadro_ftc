package org.firstinspires.ftc.teamcode.Components.Base;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveTrain {
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;

    Gamepad gamepad1;
    IMU imu;

    Telemetry telemetry;

    boolean mode = true;


    public DriveTrain(HardwareMap hardwareMap) {
        this.imu = hardwareMap.get(IMU.class, "imu");


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);


        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");

        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void init(Gamepad gamepad1, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public void power(double a, double b, double c, double d){
        motorLF.setPower(a);
        motorLB.setPower(b);
        motorRF.setPower(c);
        motorRB.setPower(d);

        telemetry.addData("[motorLF] position", motorLF.getCurrentPosition());
        telemetry.addData("[motorLB] position", motorLB.getCurrentPosition());
        telemetry.addData("[motorRF] position", motorRF.getCurrentPosition());
        telemetry.addData("[motorRB] position", motorRB.getCurrentPosition());

        telemetry.update();
    }


    public void modeA() {
        int sprint = gamepad1.right_trigger > 0 ? 1 : 2;

        if(gamepad1.dpad_up) {
            power(0.5*sprint, 0.5*sprint, 0.5*sprint, 0.5*sprint);
        } else if (gamepad1.dpad_down) {
            power(-0.5*sprint, -0.5*sprint, -0.5*sprint, -0.5*sprint);
        } else if (gamepad1.dpad_left) {
            power(-0.5*sprint, -0.5*sprint, 0.5*sprint, 0.5*sprint);
        } else if (gamepad1.dpad_right) {
            power(0.5*sprint, 0.5*sprint, -0.5*sprint, -0.5*sprint);
        }
    }

    public void modeB() {
        int sprint = gamepad1.right_trigger > 0 ? 1 : 2;

        float left_stick = gamepad1.left_stick_y;
        float right_stick = gamepad1.right_stick_y;

        power(left_stick*sprint, left_stick*sprint, right_stick*sprint, right_stick*sprint);
    }


    public void Control() {
        if(gamepad1.a) {
            mode = mode ? true : false;
        }


        if(mode) {
            modeA();
        }
        else {
            modeB();
        }
    }

    public void ControlMecanum() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorLF.setPower(frontLeftPower);
        motorLB.setPower(backLeftPower);
        motorRF.setPower(frontRightPower);
        motorRB.setPower(backRightPower);
    }
}