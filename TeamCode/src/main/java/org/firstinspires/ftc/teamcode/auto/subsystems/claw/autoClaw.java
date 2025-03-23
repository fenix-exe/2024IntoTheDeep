package org.firstinspires.ftc.teamcode.auto.subsystems.claw;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.OldAutoClawMessage;

import androidx.annotation.NonNull;

public class autoClaw {


    /*
    * This class is used to control the claw and wrist in autonomous using servo positions.
    * WILL BE UNUSED ONCE DEGREES IMPLEMENTED
     */

    //declare 3 servos
    ServoImplEx pitch;
    ServoImplEx roll;
    ServoImplEx claw;
    DownsampledWriter oldClawWriter;


    //constructor
    public autoClaw(ServoImplEx pitch, ServoImplEx roll, ServoImplEx claw){
        this.pitch = pitch;
        this.roll = roll;
        this.claw = claw;
        oldClawWriter = new DownsampledWriter("OLD CLAW INFO", 50_000_000);

    }

    //set pitch servo position
    public void setPitch(double pos){
        pitch.setPosition(pos);
    }

    //set roll servo position
    public void setRoll(double pos){
        roll.setPosition(pos);
    }

    //set claw servo position
    public void setClaw(double pos){
        claw.setPosition(pos);
    }

    /* this action sets the wrist and claw at the same time
    *  finishes when all three servos are set to position
    * nothing happens when action is finished
    */
    public class clawControl implements Action {
        private final double pitchPos;
        private final double rollPos;
        private final double clawPos;

        clawControl(double pitchPos, double rollPos, double clawPos){
            this.pitchPos = pitchPos;
            this.rollPos = rollPos;
            this.clawPos = clawPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pitch.setPosition(pitchPos);
            roll.setPosition(rollPos);
            claw.setPosition(clawPos);
            oldClawWriter.write(new OldAutoClawMessage(pitchPos, rollPos, clawPos));
            return false;
        }
    }

    public Action clawControl(double pitchPos, double rollPos, double clawPos){
        return new clawControl(pitchPos, rollPos, clawPos);
    }



}
