package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ListOfThings;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp
public class printValuesTest extends LinearOpMode {

    ListOfThings listOfThings = new ListOfThings();
    ArrayList<ListOfThings.PositionInSpace> vector = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            vector = listOfThings.SetUpListOfThings(telemetry, "/sdcard/Download/autoPositions/auto1.csv" );
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        waitForStart();
        while (opModeIsActive()) {
            for (int i = 0; i < vector.size(); i++) {
                ListOfThings.PositionInSpace position = vector.get(i);
                telemetry.addData("Vector " + (i + 1) + " X", listOfThings.getXFromList(position));
                telemetry.addData("Vector " + (i + 1) + " Y", listOfThings.getYFromList(position));
                telemetry.addData("Vector " + (i + 1) + " Heading", listOfThings.getAngleFromList(position));
            }
            telemetry.update();
        }

    }
}
