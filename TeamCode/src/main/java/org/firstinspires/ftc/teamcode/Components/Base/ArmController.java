package org.firstinspires.ftc.teamcode.Components.Base;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {
    DcMotor motorLeft;
    DcMotor motorRight;

    Grabber grabber;
    Gamepad gamepad2;
    Telemetry telemetry;


    boolean mode = false;

    public ArmController(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2) {
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");

        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;
//        this.grabber = new Grabber(hardwareMap, telemetry, gamepad2);

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


//    Setting Power for Motors
    public void power(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);

    }


//    Setting position function
    public void setPosition(int position, double power) {
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setTargetPosition(position);

        power(power);

        while(motorLeft.isBusy() && motorRight.isBusy()) {
            telemetry.addData("ArmController motorLeft position: ", motorLeft.getCurrentPosition());
            telemetry.addData("ArmController motorRight position: ", motorRight.getCurrentPosition());
            telemetry.update();
        }

        power(0);
    }


//    Manual controlling
    public void Manual() {
        if(gamepad2.dpad_up){
            telemetry.addData("Up: ", "True");

            power(0.5);

            telemetry.update();
        } else if (gamepad2.dpad_down) {
            telemetry.addData("Down: ", "True");

            power(-0.5);

            telemetry.update();
        }
        else {
            power(0);
        }


//        if(gamepad2.dpad_up) {
//            grabber.position(1);
//        } else if(gamepad2.dpad_down) {
//            grabber.position(0);
//        }


//        grabber.grabber();
    }

//    Auto control
//    public void Auto() {
//        if(gamepad2.dpad_up){
//            setPosition(1440);
//            grabber.position(0.75);
//        } else if (gamepad2.dpad_down) {
//            setPosition(0);
//            grabber.position(0);
//        }
//
//
//        grabber.grabber();
//    }

    public void Control() {
        telemetry.addData("ArmController motorLeft position: ", motorLeft.getCurrentPosition());
        telemetry.addData("ArmController motorRight position: ", motorRight.getCurrentPosition());
        telemetry.update();
        Manual();
    }
}
