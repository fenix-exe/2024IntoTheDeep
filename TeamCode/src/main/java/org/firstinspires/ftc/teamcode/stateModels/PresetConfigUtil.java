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
    public static void loadPresetsFromConfig(){

        try (BufferedReader reader = new BufferedReader(new FileReader(PRESETFILE))){
            String line;
            while ((line = reader.readLine()) != null){
                PresetPositions presetPosition = PresetConfigUtil.fromString(line);
                if(presetPosition != null) {
                    updatePresetPosition(presetPosition);
                }
            }
        } catch (Exception e){
            LoggerUtil.logException("updatePresets",e);
        }
    }
    private static void updatePresetPosition(PresetPositions presetPosition){
        switch (presetPosition.getName()) {
            case INTAKE:
                PresetPositions.INTAKE_POSITION = presetPosition;
                break;
            case DEPOSIT_FRONT_TOP:
                PresetPositions.DEPOSIT_FRONT_TOP_POSITION = presetPosition;
                break;
            case SAFE_DRIVE:
                PresetPositions.SAFE_DRIVING_POSITION = presetPosition;
                break;
            default:
                LoggerUtil.error("updatePresets", "Read Unknown Preset," + presetPosition.toString());
                break;
        }
    }
    private static PresetPositions fromString(String csv){
        String[] readValues=csv.split(",");
        try {
            PresetPositions.PositionName name = PresetPositions.PositionName.valueOf(readValues[0]);
            double pitch = Double.parseDouble(readValues[1]);
            double roll = Double.parseDouble(readValues[2]);
            double elbowAngle = Double.parseDouble(readValues[3]);
            double slideExtension = Double.parseDouble(readValues[4]);
            return new PresetPositions(name, pitch, roll, elbowAngle, slideExtension);
        }catch(Exception e){
            LoggerUtil.logException("updatePresets",e);
        }
        return null;
    }
    public static void writePresetsToConfig(){
        File file = new File(PRESETFILE);
        try{
            file.getParentFile().mkdirs();
            file.createNewFile();
            PrintWriter writer = new PrintWriter(new FileOutputStream(file, false));

            //write presets
            writer.println(PresetPositions.INTAKE_POSITION.toString());
            writer.println(PresetPositions.DEPOSIT_FRONT_TOP_POSITION.toString());
            writer.println(PresetPositions.SAFE_DRIVING_POSITION.toString());

            //close writer
            writer.close();
        } catch (Exception e){
            LoggerUtil.logException("updatePresets",e);
        }
    }
}

