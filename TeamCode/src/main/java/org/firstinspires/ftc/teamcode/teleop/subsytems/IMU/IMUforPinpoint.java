package org.firstinspires.ftc.teamcode.teleop.subsytems.IMU;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUforPinpoint implements IIMU {
    GoBildaPinpointDriverRR driver;
    double offset = -Math.PI/2;
    public IMUforPinpoint(GoBildaPinpointDriverRR driver){
        this.driver = driver;
    }

    @Override
    public void update() {
        driver.update(GoBildaPinpointDriverRR.readData.ONLY_UPDATE_HEADING);
    }

    @Override
    public double getYaw() {
        return driver.getPosition().getHeading(AngleUnit.RADIANS) - offset;
    }

    @Override
    public void resetYaw() {
        driver.resetYaw();
        offset = 0;
    }
}
