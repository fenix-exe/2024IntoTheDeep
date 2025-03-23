package org.firstinspires.ftc.teamcode.teleop.subsytems.localization;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;

import java.util.HashMap;

public class Localization {
    private final GoBildaPinpointDriverRR pinpoint;
    private final IIMU imu;
    public Localization(GoBildaPinpointDriverRR pinpoint, IIMU imu){
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
        return imu.getYaw();
    }
    public HashMap getDebugInfo(){
        HashMap debugList = new HashMap();
        debugList.put("x", String.valueOf(getX()));
        debugList.put("y", String.valueOf(getY()));
        debugList.put("h", String.valueOf(getH()));
        return debugList;
    }
}
