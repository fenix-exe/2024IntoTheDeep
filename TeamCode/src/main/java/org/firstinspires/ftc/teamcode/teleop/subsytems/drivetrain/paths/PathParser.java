package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

import java.io.BufferedReader;
import java.io.FileReader;

public class PathParser {
    public static void readPathChains(String filename){
        try {
            BufferedReader br = new BufferedReader(new FileReader(filename));
            String line;
            boolean firstLine = true;
            while ((line = br.readLine()) != null) {
                if (firstLine){
                    //throwing away first line
                    firstLine=false;
                    continue;
                }
                String[] values = line.split(",");
                if (values[0].equals("SubmersibleToBucketBlueAlliance")){
                    switch(values[1]){
                        case "ControlPoint":
                            SubmersibleToBucketBlueAlliance.controlPoints.add(new Point(Double.parseDouble(values[2]), Double.parseDouble(values[3]), Point.CARTESIAN));
                            break;
                        case "EndPoint":
                            SubmersibleToBucketBlueAlliance.endPoint = new Point(Double.parseDouble(values[2]), Double.parseDouble(values[3]), Point.CARTESIAN);
                            break;
                        case "InterpolationType":
                            SubmersibleToBucketBlueAlliance.interpolationType = values[2];
                            break;
                        case "InterpolationParam1":
                            SubmersibleToBucketBlueAlliance.interpolationParam1 = Double.parseDouble(values[2]);
                            break;
                        case "InterpolationParam2":
                            SubmersibleToBucketBlueAlliance.interpolationParam2 = Double.parseDouble(values[2]);
                            break;
                    }
                }
            }
        } catch (Exception e) {
            LoggerUtil.logException("PresetReading",e);
        }
    }
}
