package org.firstinspires.ftc.teamcode.subsytems.slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class slideTest extends LinearOpMode {
    DcMotorEx slide;
    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        waitForStart();
        while (opModeIsActive()){
            slide.setPower(0.5);
        }
    }
}
