package org.firstinspires.ftc.teamcode.auto.subsystems.barkerIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ActiveIntakeMessage;
import org.firstinspires.ftc.teamcode.common.PinchRollerIntake;

import androidx.annotation.NonNull;

public class autoPinchRoller extends PinchRollerIntake {

    private final DownsampledWriter rollerWriter;

    public autoPinchRoller(CRServoImplEx intake) {
        super(intake);
        rollerWriter = new DownsampledWriter("BARKER INTAKE INFO", 50_000_000);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public class pinchRollerPower implements Action {
        private final double power;

        public pinchRollerPower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPower(power);
            rollerWriter.write(new ActiveIntakeMessage(power));
            return false;
        }
    }

    public Action pinchRollerPower(double power) {
        return new pinchRollerPower(power);
    }

}
