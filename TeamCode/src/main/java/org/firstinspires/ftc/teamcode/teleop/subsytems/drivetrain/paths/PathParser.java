package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

import java.io.BufferedReader;
import java.io.FileReader;

public class PathParser {
    public static String PRESETFILE = "/sdcard/Download/teleop/PathConfiguration.csv";
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
                if (values[0].equals("SubmersibleToBucket")){
                    SubmersibleToBucket dummyPath=SubmersibleToBucket.getInstance();
                    dummyPath.parse(dummyPath,values);
                }
                if (values[0].equals("SubmersibleToHumanPlayer")){
                    SubmersibleToHumanPlayer dummyPath=SubmersibleToHumanPlayer.getInstance();
                    dummyPath.parse(dummyPath,values);
                }
                if (values[0].equals("ClipPath")){
                    ClipPath dummyPath=ClipPath.getInstance();
                    dummyPath.parse(dummyPath,values);
                }
                if (values[0].equals("ClipToHumanPlayer")){
                    ClipToHumanPlayer dummyPath=ClipToHumanPlayer.getInstance();
                    dummyPath.parse(dummyPath,values);
                }
            }
        } catch (Exception e) {
            LoggerUtil.logException("PresetReading",e);
        }
    }
    public static void readPathChains(){
        readPathChains(PRESETFILE);
    }
}