package org.firstinspires.ftc.teamcode.subsytems;

public interface DriveControlMap {
    //gamepad 1 controls
    public boolean slowMode();
    //makes robot move slower, is a toggle
    public boolean driveTypeSwitch();
    //switches robot modes, is a toggle
    public boolean resetIMU();
    public boolean emergencyStop();
    //turns off robot completely
    public boolean undoEmergencyStop();
    //turns robot back on
    //gamepad 2 controls
    public boolean slidesFullyUp();
    //slides go to max
    public boolean slidesFullyDown();
    //slides go to min
    public boolean pivotParallel();
    //pivot parallel to ground
    public boolean pivotPerp();
    //pivot perpendicular to ground
    public double pivotJoystick();
    public double slideMovement();
    //joystick control of slides
    public double degreeOfFreedomX();
    //controls x degree of freedom
    public double degreeOfFreedomY();
    //controls y degree of freedom
    public boolean intakePower();
    //turns intake on/off, toggle
    public boolean intakeDirection();
    //changes intake direction, toggle
    public float intakenewForward();
    //instead of toggle, hold
    public float intakenewBackward();
    //instead of toggle, hold

    public boolean resetWrist();
    public boolean submersibleIntakeReady();
    public boolean acsent1Park();
    public boolean drivingPos();
    public boolean depositReadyBackTopBucket();
    public boolean depositReadyFrontTopBucket();
    public boolean depositReadyBackBottomBucket();
    public boolean depositReadyFrontBottomBucket();




}
