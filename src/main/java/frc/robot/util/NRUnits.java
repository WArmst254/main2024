package frc.robot.util;

public class NRUnits {

    // Constrains the given angle to [-180, 180] 
    public static double constrainDeg(double angle){
        return 360./(2*Math.PI)*constrainRad(angle*(2*Math.PI)/360);
    }


    public static double constrainRad(double angle) {
        double temp = (angle % (2*Math.PI) + (2*Math.PI)) % (2*Math.PI);
        if(temp <= (2*Math.PI)/2) return temp;
        return temp - (2*Math.PI);
    }

    public static double logConstrainRad(double angle) {
        angle = constrainRad(angle);
        if(angle < 0) angle += (2*Math.PI)/2;
        return angle;
    }
}
