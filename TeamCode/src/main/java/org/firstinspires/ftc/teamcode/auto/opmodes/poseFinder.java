package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.extractOffsets;

import java.io.IOException;

import static org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive.PARAMS;

@Config
@TeleOp(name = "POSE FINDER")
public class poseFinder extends LinearOpMode {



    //ftc dashboard values to set pinpoint
    public static double x = 39.7;
    public static double y = 65;
    public static double heading = -180;
    public GoBildaPinpointDriverRR pinpoint;


    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,PARAMS.pinpointDeviceName);
        extractOffsets offsets = new extractOffsets();

        try {
            offsets.offsetGetter("/sdcard/Download/autoOffsets/clipOffsets.csv");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }


        //set up ftc dashboard telemetry
        MultipleTelemetry multi = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // reset pinpoint and calibrate
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(new Pose2d(x,y,Math.toRadians(heading)));


        waitForStart();


        while (opModeIsActive()) {
            //update pinpoint telemetry
            pinpoint.update();

            //display pinpoint position on dashboard and ds
            multi.addData("pose x", pinpoint.getPositionRR().position.x);
            multi.addData("pose y", pinpoint.getPositionRR().position.y);
            multi.addData("pose heading", Math.toDegrees(pinpoint.getPositionRR().heading.toDouble()));
            multi.addLine("" + offsets.getPitchOffset());
            multi.update();

            /*
            if (gamepad1.a) {
                Arrays.stream(new File("/sdcard/Download/autoLogger").listFiles()).forEach(File::delete);
            }*/
        }

    }
}
