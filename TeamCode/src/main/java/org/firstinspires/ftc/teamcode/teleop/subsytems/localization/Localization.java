package org.firstinspires.ftc.teamcode.teleop.subsytems.localization;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;

public class Localization {
    private GoBildaPinpointDriverRR pinpoint;
    private IMU imu;
    public Localization(GoBildaPinpointDriverRR pinpoint, IMU imu){
        this.pinpoint = pinpoint;
        this.imu = imu;
    }
    public double getX(){
        return pinpoint.getPosX();
    }
    public double getY(){
        return pinpoint.getPosY();
    }
    public double getH(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public HashMap getDebugInfo(){
        HashMap debugList = new HashMap();
        debugList.put("x", String.valueOf(getX()));
        debugList.put("y", String.valueOf(getY()));
        debugList.put("h", String.valueOf(getH()));
        return debugList;
    }
}
