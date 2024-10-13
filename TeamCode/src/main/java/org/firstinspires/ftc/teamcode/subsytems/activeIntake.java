package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class activeIntake {
    Gamepad gamepad2;
    Gamepad gamepad2previous;
    CRServo intake;

    public activeIntake(Gamepad gamepad2, Gamepad gamepad2previous, CRServo intake){
        this.gamepad2=gamepad2;
        this.gamepad2previous=gamepad2previous;
        this.intake=intake;
    }

    
}
