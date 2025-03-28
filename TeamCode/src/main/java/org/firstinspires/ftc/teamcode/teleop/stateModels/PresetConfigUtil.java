package org.firstinspires.ftc.teamcode.teleop.stateModels;

import org.firstinspires.ftc.teamcode.teleop.util.LoggerUtil;

import java.io.BufferedReader;
import java.io.FileReader;

public class PresetConfigUtil {
    public static String PRESETFILE = "/sdcard/Download/teleop/PresetPositions.csv";
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
            case "INTAKE_STATE_MODEL_PARAMS":
                StateModelParameters.IntakeStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.IntakeStateParameters.roll = Double.parseDouble(configValues[2]);
                StateModelParameters.IntakeStateParameters.downPitch = Double.parseDouble(configValues[3]);
                StateModelParameters.IntakeStateParameters.downRoll = Double.parseDouble(configValues[4]);
                StateModelParameters.IntakeStateParameters.elbowAngle = Double.parseDouble(configValues[5]);
                StateModelParameters.IntakeStateParameters.slideLength = Double.parseDouble(configValues[6]);
                break;
            case "DEPOSIT_STATE_MODEL_PARAMS":
                StateModelParameters.DepositStateParameters.intermediatePitch = Double.parseDouble(configValues[1]);
                StateModelParameters.DepositStateParameters.pitch = Double.parseDouble(configValues[2]);
                StateModelParameters.DepositStateParameters.roll = Double.parseDouble(configValues[3]);
                StateModelParameters.DepositStateParameters.elbowAngle = Double.parseDouble(configValues[4]);
                StateModelParameters.DepositStateParameters.slideLength = Double.parseDouble(configValues[5]);
                StateModelParameters.DepositStateParameters.slideRetractionLength = Double.parseDouble(configValues[6]);
                break;
            case "DEPOSIT_SAMPLE_INTO_BUCKET_STATE_MODEL_PARAMS":
                StateModelParameters.DepositSampleIntoBucketStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.DepositSampleIntoBucketStateParameters.roll = Double.parseDouble(configValues[2]);
                StateModelParameters.DepositSampleIntoBucketStateParameters.elbowAngle = Double.parseDouble(configValues[3]);
                StateModelParameters.DepositSampleIntoBucketStateParameters.intermediateElbowAngle = Double.parseDouble(configValues[4]);
                StateModelParameters.DepositSampleIntoBucketStateParameters.slideLength = Double.parseDouble(configValues[5]);
                break;
            case "GRAB_BLOCK_FROM_OUTSIDE_STATE_MODEL_PARAMS":
                StateModelParameters.GrabBlockFromOutsideStateParameters.downPitch = Double.parseDouble(configValues[1]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.upPitch= Double.parseDouble(configValues[2]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.upRoll = Double.parseDouble(configValues[3]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeDownAngleCloseSlides = Double.parseDouble(configValues[4]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeDownAngleMediumSlides = Double.parseDouble(configValues[5]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeDownAngleFarSlides = Double.parseDouble(configValues[6]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.elbowIntakeUpAngle = Double.parseDouble(configValues[7]);
                StateModelParameters.GrabBlockFromOutsideStateParameters.waitTime = Double.parseDouble(configValues[8]);
                break;
            case "PICKUP_SPECIMENS_STATE_MODEL_PARAMS":
                StateModelParameters.PickupSpecimensStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.PickupSpecimensStateParameters.roll= Double.parseDouble(configValues[2]);
                StateModelParameters.PickupSpecimensStateParameters.elbowAngle = Double.parseDouble(configValues[3]);
                StateModelParameters.PickupSpecimensStateParameters.slideLength = Double.parseDouble(configValues[4]);
                StateModelParameters.PickupSpecimensStateParameters.elbowUpAngle = Double.parseDouble(configValues[5]);
                StateModelParameters.PickupSpecimensStateParameters.pickupSlideLength = Double.parseDouble(configValues[6]);
                StateModelParameters.PickupSpecimensStateParameters.endSlideLength = Double.parseDouble(configValues[7]);
                StateModelParameters.PickupSpecimensStateParameters.pitchEnd = Double.parseDouble(configValues[8]);
                StateModelParameters.PickupSpecimensStateParameters.rollEnd = Double.parseDouble(configValues[9]);
                break;
            case "DEPOSIT_SPECIMENS_STATE_MODEL_PARAMS":
                StateModelParameters.DepositSpecimensStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.DepositSpecimensStateParameters.roll= Double.parseDouble(configValues[2]);
                StateModelParameters.DepositSpecimensStateParameters.elbowAngle = Double.parseDouble(configValues[3]);
                StateModelParameters.DepositSpecimensStateParameters.slideLength = Double.parseDouble(configValues[4]);
                break;
            case "DROP_BLOCK_AND_MOVE_WRIST_DOWN_STATE_MODEL_PARAMS":
                StateModelParameters.DropBlockAndMoveWristDown.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.DropBlockAndMoveWristDown.elbowAngle = Double.parseDouble(configValues[2]);
                StateModelParameters.DropBlockAndMoveWristDown.slideLength = Double.parseDouble(configValues[3]);
                break;
            case "HANG_STATE_MODEL_PARAMS":
                StateModelParameters.Hang.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.Hang.roll = Double.parseDouble(configValues[2]);
                StateModelParameters.Hang.linearActuatorExtension = Double.parseDouble(configValues[3]);
                StateModelParameters.Hang.linearActuatorRetraction = Double.parseDouble(configValues[4]);
                StateModelParameters.Hang.initialElbowAngle = Double.parseDouble(configValues[5]);
                StateModelParameters.Hang.slideExtension = Double.parseDouble(configValues[6]);
                StateModelParameters.Hang.hangElbowAngle = Double.parseDouble(configValues[7]);
                StateModelParameters.Hang.slideIntermediatePosition = Double.parseDouble(configValues[8]);
                StateModelParameters.Hang.slideRetraction = Double.parseDouble(configValues[9]);
                StateModelParameters.Hang.endElbowAngle = Double.parseDouble(configValues[10]);
                break;
            case "DEPOSIT_SAMPLE_INTO_OBSERVATION_ZONE_STATE_MODEL_PARAMS":
                StateModelParameters.DepositSampleIntoObservationZone.retractionLength = Double.parseDouble(configValues[1]);
                StateModelParameters.DepositSampleIntoObservationZone.pitchDown = Double.parseDouble(configValues[2]);
                StateModelParameters.DepositSampleIntoObservationZone.extensionLength = Double.parseDouble(configValues[3]);
                StateModelParameters.DepositSampleIntoObservationZone.downPitch = Double.parseDouble(configValues[4]);
                StateModelParameters.DepositSampleIntoObservationZone.downRoll = Double.parseDouble(configValues[5]);
                break;
            case "ENTER_SUBMERSIBLE_STATE_MODEL_PARAMS":
                StateModelParameters.EnterSubmersibleStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.EnterSubmersibleStateParameters.waitTime = Double.parseDouble(configValues[2]);
                break;
            case "LEAVE_SUBMERSIBLE_STATE_MODEL_PARAMS":
                StateModelParameters.LeaveSubmersibleStateParameters.pitch = Double.parseDouble(configValues[1]);
                StateModelParameters.LeaveSubmersibleStateParameters.slideLength = Double.parseDouble(configValues[2]);
            default:
                LoggerUtil.error("updatePresets", "Read Unknown Preset," + preset_name );
                break;
        }
    }
}

