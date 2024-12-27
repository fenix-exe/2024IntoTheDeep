package org.firstinspires.ftc.teamcode.util.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
@Disabled
public class ActiveIntakeColorSensorCalibration extends LinearOpMode {

    private DistanceSensor color_DistanceSensor;
    private RevColorSensorV3 colorSensor;
    public static boolean record =  false;
    public static String color = "Yellow";
    ElapsedTime timer;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {


        color_DistanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        timer = new ElapsedTime();
        timer.reset();

        // Put initialization blocks here.

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here
                telemetry.addData("MM distance", color_DistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Red hue", colorSensor.red());
                telemetry.addData("Blue hue", colorSensor.blue());
                telemetry.addData("Green hue", colorSensor.green());
                if (timer.milliseconds()>500){
                    timer.reset();
                    if (record) {
                        writeColorToFile("/sdcard/FIRST/colorTestdata.csv", colorSensor.red(), colorSensor.blue(), colorSensor.green(), telemetry);
                        telemetry.addLine("written to file");
                    }
                }


                telemetry.update();
            }
        }
    }
    private void writeColorToFile(String file, int red, int blue, int green, Telemetry telemetry){
        ArrayList list = new ArrayList();
        list.add(color);
        list.add(Integer.toString(red));
        list.add(Integer.toString(green));
        list.add(Integer.toString(blue));
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(file, true )); // true makes sure that the file is appended to
            writer.write(String.join(",", list));
            writer.newLine();
            writer.close();

        } catch (Exception e){
            telemetry.addLine(e.getMessage());
        }
    }
}