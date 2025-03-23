package org.firstinspires.ftc.teamcode.teleop.subsytems.linearActuator;


import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.CommonLinearActuator;

public class LinearActuator extends CommonLinearActuator {
    public LinearActuator(DcMotorEx linearActuatorMotor, RevTouchSensor limitSwitch){
        super(linearActuatorMotor, limitSwitch);
    }
}
