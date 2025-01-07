package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU;

public interface IIMU {

    public void update();
    //get yaw in radians
    public double getYaw();
    //reset imu
    public void resetYaw();
}
