package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Base.ArmController;
import org.firstinspires.ftc.teamcode.Components.Base.DriveTrain;

@TeleOp(name = "Quadro_ftc")
public class Main extends LinearOpMode {
    DriveTrain drive;
    ArmController armController;
    @Override
    public void runOpMode() {

        waitForStart();
        while(opModeIsActive()) {
            armController.Control();
            drive.Control();
        }
    }
}
