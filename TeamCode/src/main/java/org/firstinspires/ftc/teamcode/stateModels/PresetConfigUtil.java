package org.firstinspires.ftc.teamcode.stateModels;

import org.firstinspires.ftc.teamcode.modules.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.modules.arm.ArmPresetPositionNames;
import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffectorPresetPosition;
import org.firstinspires.ftc.teamcode.modules.endEffectorV1.EndEffectorPresetPositionNames;
import org.firstinspires.ftc.teamcode.util.LoggerUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintWriter;

public class PresetConfigUtil {
    public static String PRESETFILE = "/sdcard/Download/TeleOpV5/PresetPositions.csv";
    public static int loadPresetsFromConfig(){
        int count  = 0;   // counts how many different preset types we read from the file
        try (BufferedReader reader = new BufferedReader(new FileReader(PRESETFILE))){
            String line;
            while ((line = reader.readLine()) != null){
                updatePresetPosition(line.split(","));
                count++;
            }
        } catch (Exception e){
            LoggerUtil.logException("updatePresets",e);
        }
        return count;
    }
    private static void updatePresetPosition(String[] configValues){
        String preset_name = configValues[0];
        switch (preset_name) {
            case "DRIVE_STATE_MODEL_PARAMS":
                StateModelParameters.DriveStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.DriveStateParameters.elbowAngle = Double.parseDouble(configValues[2]);
                StateModelParameters.DriveStateParameters.slideLength = Double.parseDouble(configValues[3]);
                break;
            default:
                LoggerUtil.error("updatePresets", "Read Unknown Preset," + preset_name );
                break;
        }
    }
}

