package org.firstinspires.ftc.teamcode.util.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.extractAuto;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

@Disabled
public class printValuesTest extends LinearOpMode {

    extractAuto extractAuto = new extractAuto();
    ArrayList<extractAuto.PositionInSpace> vector = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            vector = extractAuto.SetUpListOfThings(telemetry, "/sdcard/Download/autoPositions/auto1.csv" );
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (opModeIsActive()) {
            for (int i = 0; i < vector.size(); i++) {
                extractAuto.PositionInSpace position = vector.get(i);
                telemetry.addData("Vector " + (i + 1) + " X", extractAuto.getXFromList(position));
                telemetry.addData("Vector " + (i + 1) + " Y", extractAuto.getYFromList(position));
                telemetry.addData("Vector " + (i + 1) + " Heading", extractAuto.getAngleFromList(position));
            }
            telemetry.update();
        }

    }
}
