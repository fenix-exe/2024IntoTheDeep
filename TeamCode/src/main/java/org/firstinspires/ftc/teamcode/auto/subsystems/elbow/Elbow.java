package org.firstinspires.ftc.teamcode.auto.subsystems.elbow;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ElbowMessage;
import org.firstinspires.ftc.teamcode.common.CommonElbow;

import androidx.annotation.NonNull;


public class Elbow extends CommonElbow {

    /*
    * This class allows for control of the elbow using degrees in autonomous
    */

    public DcMotorEx elbowMotor;
    public RevTouchSensor limitSwitch;
    private final DownsampledWriter elbowWriter;

    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, int topPosition){
        super(elbow, limitSwitch, topPosition);
        this.elbowMotor = elbow;
        elbowWriter = new DownsampledWriter("ELBOW INFO", 50_000_000);
    }


    /* this action sets elbow motor position using degrees
    * finishes when elbow is within 0.5 degrees of the position
    * elbow stops moving when the action finishes
     */
    public class elbowControl implements Action {
        private final double target;
        private final double speed;
        private boolean initialized = false;

        elbowControl(double targetPos, double speed) {
            this.target = targetPos;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                setTargetAngleAndSpeed(target, speed);
                initialized = true;
            }

            //elbowWriter.write(new ElbowMessage(getElbowAngle(), target, elbowMotor.getCurrent(CurrentUnit.MILLIAMPS)));

            if (getElbowTargetAngle()-1.5 < getElbowAngle() && getElbowAngle() < getElbowTargetAngle()+1.5) {
                return false;
            } else {
                return true;
            }
        }


    }
    public Action elbowControl(double targetPos, double speed) {
        return new elbowControl(targetPos, speed);
    }



}
