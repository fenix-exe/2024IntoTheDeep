package org.firstinspires.ftc.teamcode.subsytems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

import androidx.annotation.NonNull;

public class Claw {
    Servo claw;

    public Claw (Servo claw){
        this.claw = claw;
    }
    public void openClaw(){
        claw.setPosition(RobotConstants.OPEN_POSITION);
    }
    public void closeClaw(){claw.setPosition(RobotConstants.CLOSED_POSITION);}
    public void intermediateClaw(){claw.setPosition(RobotConstants.INTERMEDIATE_POSITION);}

    public class clawControl implements Action {
        private final double clawPos;

        public clawControl (double clawPos){
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(clawPos);
            return false;
        }
    }

    public Action clawControl(double clawPos){
        return new clawControl(clawPos);
    }
}
