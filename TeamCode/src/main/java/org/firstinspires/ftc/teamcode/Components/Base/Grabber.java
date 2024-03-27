package org.firstinspires.ftc.teamcode.Components.Base;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber {
    Servo grabberMain;
    Servo grabberLeft;
    Servo grabberRight;
    Telemetry telemetry;
    Gamepad gamepad2;


    boolean leftBumper = false;
    boolean rightBumper = false;

    public Grabber(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad2 = gamepad2;

        grabberMain = hardwareMap.get(Servo.class, "grabberMain");
        grabberLeft = hardwareMap.get(Servo.class, "grabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "grabberRight");

        grabberRight.setDirection(Servo.Direction.REVERSE);
    }


    public void position(double position) {
        grabberMain.setPosition(position);
    }

    public void grabber() {
        if(gamepad2.left_bumper) {
            leftBumper = leftBumper ? false : true;
        } else if (gamepad2.right_bumper) {
            rightBumper = rightBumper ? false : true;
        }


        grabberLeft.setPosition(leftBumper ? 0.5 : 0);
        grabberRight.setPosition(rightBumper ? 0.5 : 0);
    }

}
