package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUforREV implements IIMU {
    IMU imu;
    public IMUforREV(IMU imu){
        this.imu=imu;
    }

    @Override
    public void update() {

    }

    @Override
    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public void resetYaw() {
        imu.resetYaw();
    }
}
