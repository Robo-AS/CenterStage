package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.hardware.DcMotor;

public class MotorValues {
    public double fl,fr,bl,br, globalMultiplier=0.5;

    public MotorValues(double cfl, double cfr, double cbl, double cbr){
        fl=cfl;
        fr=cfr;
        bl=cbl;
        br=cbr;
    }

    public void NormaliseValues(){
        double maxSpeed = Math.max(Math.max(Math.abs(fr),Math.abs(fr)), Math.max(Math.abs(br), Math.abs(bl)));
        
        if (maxSpeed<1) return;
        
        fl/=maxSpeed;
        fr/=maxSpeed;
        bl/=maxSpeed;
        br/=maxSpeed;
    }

    public void SlowMode(){
        fl/=2;
        fr/=2;
        bl/=2;
        br/=2;
    }
    
}