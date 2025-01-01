package org.firstinspires.ftc.teamcode.robot;

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

public class ConfigUtil {
    public static String PRESETFILE = "/sdcard/Download/TeleOp/PresetPositions.csv";
    public static void loadPresetsFromConfig(){

        try (BufferedReader reader = new BufferedReader(new FileReader(PRESETFILE))){
            String line;
            while ((line = reader.readLine()) != null){
                Object presetPosition = ConfigUtil.fromString(line);
                if(presetPosition != null) {
                    if (presetPosition instanceof ArmPresetPosition) {
                        updateArmPreset((ArmPresetPosition) presetPosition);
                    } else if (presetPosition instanceof EndEffectorPresetPosition){
                        updateEndEffectorPreset((EndEffectorPresetPosition) presetPosition);
                    }

                }
            }
        } catch (Exception e){
            LoggerUtil.logException(e);
        }
    }

    private static void updateArmPreset(ArmPresetPosition presetPosition) {
        switch (presetPosition.getName()) {
            case INTAKE:
                ArmPresetPosition.INTAKE_POSITION = presetPosition;
                break;
            case DEPOSIT_FRONT_TOP:
                ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_FRONT_BOTTOM:
                ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_BACK_TOP:
                ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_BACK_BOTTOM:
                ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = presetPosition;
                break;
            case DRIVING:
                ArmPresetPosition.SAFE_DRIVING_POSITION = presetPosition;
                break;
            case ASCENT_2_HANG:
                ArmPresetPosition.ASCENT_2_HANG = presetPosition;
                break;
            case FLAT_ELBOW:
                ArmPresetPosition.FLAT_ELBOW = presetPosition;
                break;
            case INTAKE_DOWN:
                ArmPresetPosition.INTAKE_DOWN = presetPosition;
                break;
            default:
                LoggerUtil.error("configUtil", "Read Unknown Arm Preset," + presetPosition.toString());
                break;
        }
    }

    private static void updateEndEffectorPreset(EndEffectorPresetPosition presetPosition) {
        switch (presetPosition.getName()) {
            case INTAKE:
                EndEffectorPresetPosition.INTAKE_POSITION = presetPosition;
                break;
            case DEPOSIT_FRONT_TOP:
                EndEffectorPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_FRONT_BOTTOM:
                EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_BACK_TOP:
                EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = presetPosition;
                break;
            case DEPOSIT_BACK_BOTTOM:
                EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = presetPosition;
                break;
            case DRIVING:
                EndEffectorPresetPosition.SAFE_DRIVING_POSITION = presetPosition;
                break;
            case DISPERSION:
                EndEffectorPresetPosition.DISPERSION = presetPosition;
                break;
            case INTAKE_DOWN:
                EndEffectorPresetPosition.INTAKE_DOWN = presetPosition;
                break;
            default:
                LoggerUtil.error("configUtil", "Read Unknown End Effector Preset," + presetPosition.toString());
                break;
        }
    }

    private static Object fromString(String csv){
        String[] readValues=csv.split(",");
        try {
            String typeOfPreset = readValues[0];
            if (typeOfPreset.equals("ARM_PRESET")){
                ArmPresetPositionNames name = ArmPresetPositionNames.valueOf(readValues[1]);
                double elbowAngle = Double.parseDouble(readValues[2]);
                double slideExtension = Double.parseDouble(readValues[3]);
                return new ArmPresetPosition(name, elbowAngle, slideExtension);
            } else if (typeOfPreset.equals("END_EFFECTOR_PRESET")) {
                EndEffectorPresetPositionNames name = EndEffectorPresetPositionNames.valueOf(readValues[1]);
                double pitch = Double.parseDouble(readValues[2]);
                double roll = Double.parseDouble(readValues[3]);
                return new EndEffectorPresetPosition(name, pitch, roll);
            }
        }catch(Exception e){
            LoggerUtil.logException(e);
        }
        return null;
    }
    public static void writePresetsToConfig(){
        File file = new File(PRESETFILE);
        try{
            file.getParentFile().mkdirs();
            file.createNewFile();
            PrintWriter writer = new PrintWriter(new FileOutputStream(file, false));
            //writing arm presets
            writer.println("ARM_PRESET," + ArmPresetPosition.INTAKE_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.SAFE_DRIVING_POSITION.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.ASCENT_2_HANG.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.FLAT_ELBOW.toString());
            writer.println("ARM_PRESET," + ArmPresetPosition.INTAKE_DOWN.toString());

            //writing end effector presets
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.INTAKE_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.SAFE_DRIVING_POSITION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.DISPERSION.toString());
            writer.println("END_EFFECTOR_PRESET," + EndEffectorPresetPosition.INTAKE_DOWN.toString());

            writer.close();
        } catch (Exception e){
            LoggerUtil.logException(e);
        }
    }
}
