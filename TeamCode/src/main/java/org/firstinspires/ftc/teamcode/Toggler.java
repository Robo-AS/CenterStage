package org.firstinspires.ftc.teamcode;

public class Toggler {

    public enum STATUS{
        NOT_PRESSED,
        JUST_PRESSED,
        PRESSED,
        JUST_RELEASED
    }

    STATUS status = STATUS.NOT_PRESSED;
    public void update(boolean value)
    {
        if(value){
            if(status==STATUS.NOT_PRESSED){
                status=STATUS.JUST_PRESSED;
            }else{
                status=STATUS.PRESSED;
            }
        }
        else{
            if(status==STATUS.PRESSED){
                status=STATUS.JUST_RELEASED;
            }else{
                status=STATUS.NOT_PRESSED;
            }

        }
    }
}