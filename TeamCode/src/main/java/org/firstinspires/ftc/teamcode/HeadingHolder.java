package org.firstinspires.ftc.teamcode;

//Class created to store the last known heading of the gyro
public class HeadingHolder {

    //lastHeading, accessed by get and setHeading. Starts at 0.
    private static int lastHeading = 0;

    //Setter method
    public static void setHeading(int aHeading){
        lastHeading = aHeading;
    }

    //Getter method
    public static int getHeading(){
        return lastHeading;
    }
}
