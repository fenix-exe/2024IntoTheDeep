package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforPinpoint;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforREV;
@TeleOp
public class IMUtest extends LinearOpMode {
    IIMU RevIMU;
    IIMU PinpointIMU;
    @Override
    public void runOpMode() throws InterruptedException {
        IMU revIMU = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters= new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        revIMU.initialize(parameters);
        RevIMU = new IMUforREV(revIMU);
        GoBildaPinpointDriver pinpointIMU = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        PinpointIMU = new IMUforPinpoint(pinpointIMU);


        waitForStart();

        while (opModeIsActive()){
            PinpointIMU.update();
            if (gamepad1.a){
                RevIMU.resetYaw();
                PinpointIMU.resetYaw();
            }
            if (gamepad1.b){
                RevIMU.resetYaw();
            }
            if (gamepad1.x){
                PinpointIMU.resetYaw();
            }
            telemetry.addData("REV IMU ANGLE", Math.toDegrees(RevIMU.getYaw()));
            telemetry.addData("PINPOINT IMU ANGLE", Math.toDegrees(PinpointIMU.getYaw()));
            telemetry.update();
        }
    }
}
