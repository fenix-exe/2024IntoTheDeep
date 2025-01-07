package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU;

public class IMUforPinpoint implements IIMU {
    GoBildaPinpointDriver driver;
    public IMUforPinpoint(GoBildaPinpointDriver driver){
        this.driver = driver;
    }
    @Override
    public double getYaw() {
        return driver.getHeading();
    }

    @Override
    public void resetYaw() {
        driver.recalibrateIMU();
    }
}
