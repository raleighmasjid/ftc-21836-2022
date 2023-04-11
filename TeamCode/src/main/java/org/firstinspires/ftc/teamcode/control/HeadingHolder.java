package org.firstinspires.ftc.teamcode.control;

//Class created to store the last known heading of the gyro
public class HeadingHolder {

    //lastHeading, accessed by get and setHeading. Starts at 0.
    private static double lastHeading = 0.0;

    //Setter method
    public static void setHeading(double newHeading){
        lastHeading = newHeading;
    }

    //Getter method
    public static double getHeading(){
        return lastHeading;
    }
}
