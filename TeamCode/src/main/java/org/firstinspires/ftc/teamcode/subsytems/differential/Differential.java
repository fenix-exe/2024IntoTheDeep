package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.robotcore.hardware.PwmControl;

import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class Differential {
    ServoImplEx left;
    ServoImplEx right;

    float leftRange = 270;
    float rightRange = 270;
    public static float pitchError = -15;
    public static float rollError = 30;

    private double last_known_pitch = 0;
    private double last_known_roll = 0;

    public Differential(ServoImplEx left, ServoImplEx right){
        this.left = left;
        this.right = right;
        left.setPwmRange(new PwmControl.PwmRange(600,2400));
        right.setPwmRange(new PwmControl.PwmRange(600,2400));
    }

    private double setAPosition(double pitch, double roll) {
        double aPos;
        aPos = (1 / leftRange) * (roll + ( pitch / 2)) + 0.5;
        return aPos;
    }

    private double setBPosition(double pitch, double roll) {
        double bPos;
        bPos = (1 / rightRange) * (roll - (pitch / 2)) + 0.5;
        return bPos;
    }

    public void setDifferentialPosition(double pitch, double roll){
        left.setPosition(setAPosition(-2*(pitch)+pitchError, roll+rollError));
        right.setPosition(setBPosition(-2*(pitch)+pitchError, roll+rollError));
        this.last_known_pitch = pitch;
        this.last_known_roll = roll;

    }
    public double returnRoll() {
        //return 135*(left.getPosition())+135*(right.getPosition())-135;
        return last_known_roll;
    }

    public double returnPitch() {
        //return 2*(135*(left.getPosition())-135*(right.getPosition())-135)-540*(right.getPosition())+270;
        return last_known_pitch;
    }

    

}
