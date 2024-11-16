package org.firstinspires.ftc.teamcode.subsytems.endeffector;

import org.firstinspires.ftc.teamcode.subsytems.activeIntake.activeIntake;
import org.firstinspires.ftc.teamcode.subsytems.differential.differential;

public class EndEffector {
    activeIntake intake;
    differential differential;

    public EndEffector(activeIntake intake, differential differential ){
        this.intake = intake;
        this.differential = differential;
    }

}
