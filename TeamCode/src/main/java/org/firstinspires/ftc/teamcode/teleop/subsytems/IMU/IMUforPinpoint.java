package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUforPinpoint implements IIMU {
    GoBildaPinpointDriver driver;
    public IMUforPinpoint(GoBildaPinpointDriver driver){
        this.driver = driver;
    }

    @Override
    public void update() {
        driver.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
    }

    @Override
    public double getYaw() {
        return driver.getPosition().getHeading(AngleUnit.RADIANS);
    }

    @Override
    public void resetYaw() {
        driver.recalibrateIMU();
    }
}
