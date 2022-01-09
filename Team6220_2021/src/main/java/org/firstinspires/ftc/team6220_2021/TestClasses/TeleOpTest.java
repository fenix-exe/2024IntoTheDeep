package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_2021.MasterTeleOp;

//@Disabled
@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends MasterTeleOp {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        motorBelt.setPower(0.5);
        motorBelt.setTargetPosition(1250);

        while (opModeIsActive()) {
            telemetry.addData("belt: ", motorBelt.getCurrentPosition());
            telemetry.update();
        }
    }
}