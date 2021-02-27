package frc.robot.utils;

public class Utilities {
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
    public static double applySensitivityFactor(double input, double factor){
        // See http://www.chiefdelphi.com/forums/showthread.php?p=921992
        // The transform can only work on values between -1 and 1.
        if (input>1) { return 1; }
        if (input <-1) { return -1; }
        // The sensitivity factor MUST be between 0 and 1!
        double capped = Math.max(Math.min(factor, 1),0);
        return capped*input*input*input + (1-capped)*input;
    }
    
    
    
    
    
}
