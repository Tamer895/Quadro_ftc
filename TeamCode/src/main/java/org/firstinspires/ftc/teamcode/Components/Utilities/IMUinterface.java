package org.firstinspires.ftc.teamcode.Components.Utilities;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.Base.DriveTrain;

import java.security.KeyPairGeneratorSpi;

public class IMUinterface {

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    DriveTrain drivetrain;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    IMU.Parameters parameters;

    Telemetry telemetry;

    IMU imu;

    public IMUinterface(HardwareMap hardwareMap, Telemetry telemetry) {
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;
        this.drivetrain = new DriveTrain(hardwareMap);

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
    }

    public void Stabilize() {
        double refrenceAngle = Math.toRadians(90);

        double power = PIDControl(refrenceAngle, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        drivetrain.power(-power, -power, power, power);
        telemetry.update();
    }

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
