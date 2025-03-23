package org.firstinspires.ftc.teamcode.auto.subsystems.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.SlideMessage;
import org.firstinspires.ftc.teamcode.commonCode.CommonSlide;

import static java.lang.Math.floor;


import androidx.annotation.NonNull;

public class Slide extends CommonSlide {
    public DcMotorEx leftSlideMotor;
    public DcMotorEx rightSlideMotor;
    public RevTouchSensor homingSwitch;
    private final DownsampledWriter slideWriter;


    public Slide(DcMotorEx leftSlideMotor, DcMotorEx rightSlideMotor, RevTouchSensor homingSwitch){
        super(leftSlideMotor, rightSlideMotor, homingSwitch);
        this.homingSwitch = homingSwitch;
        this.leftSlideMotor = leftSlideMotor;
        this.rightSlideMotor = rightSlideMotor;
        slideWriter = new DownsampledWriter("SLIDE INFO", 50_000_000);
    }


    public class slideControl implements Action {
        private final double targetPos;
        slideControl(double targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setSlideExtensionLength(targetPos);

            slideWriter.write(new SlideMessage(getSlideExtensionInInches(), targetPos));

            if (homingSwitch.isPressed()){
                leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setSlideExtensionLength(targetPos);
            }
            return !(targetPos - 0.5 < getSlideExtensionInInches()) || !(getSlideExtensionInInches() < targetPos + 0.5);
        }
    }
    public Action slideControl(double targetPos){
        return new slideControl(targetPos);
    }


}
