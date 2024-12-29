package org.firstinspires.ftc.teamcode.robot;


import org.firstinspires.ftc.teamcode.subsytems.arm.ArmPresetPosition;
import org.firstinspires.ftc.teamcode.subsytems.endeffector.EndEffectorPresetPosition;

import java.io.BufferedReader;
import java.io.FileReader;

public class ConfigReader {
    public static void readConfig(String fileName){
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(fileName));
            String line;
            reader.readLine();

            while ((line = reader.readLine()) != null){
                String[] readValues= line.split(",");
                try{
                    String position = readValues[0];
                    double elbowAngle = Double.parseDouble(readValues[1]);
                    double slideExtension = Double.parseDouble(readValues[2]);
                    double pitch = Double.parseDouble(readValues[3]);
                    double roll = Double.parseDouble(readValues[4]);
                    switch (position){
                        case "INTAKE":
                            ArmPresetPosition.INTAKE_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.INTAKE_POSITION = new EndEffectorPresetPosition(pitch, roll);
                            break;
                        case "DEPOSIT_FRONT_TOP":
                            ArmPresetPosition.DEPOSIT_FRONT_TOP_BUCKET_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(pitch, roll);
                            break;
                        case "DEPOSIT_FRONT_BOTTOM":
                            ArmPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.DEPOSIT_FRONT_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(pitch, roll);
                            break;
                        case "DEPOSIT_BACK_TOP":
                            ArmPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.DEPOSIT_BACK_TOP_BUCKET_POSITION = new EndEffectorPresetPosition(pitch, roll);
                            break;
                        case "DEPOSIT_BACK_BOTTOM":
                            ArmPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.DEPOSIT_BACK_BOTTOM_BUCKET_POSITION = new EndEffectorPresetPosition(pitch, roll);
                            break;
                        case "DRIVING_POSITION":
                            ArmPresetPosition.SAFE_DRIVING_POSITION = new ArmPresetPosition(elbowAngle, slideExtension);
                            EndEffectorPresetPosition.SAFE_DRIVING_POSITION = new EndEffectorPresetPosition(elbowAngle, slideExtension);
                            break;
                    }

                } catch(Exception e) {}
            }
        } catch (Exception e){}

    }
}
