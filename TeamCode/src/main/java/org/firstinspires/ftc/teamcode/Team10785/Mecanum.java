package org.firstinspires.ftc.teamcode.Team10785;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Mecanum {
    /**
     * Mecanum motion vector.
     */
    public static class Motion {
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public double vTheta;

        /**
         * Sets the motion to the given values.
         */
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }
    }

    /**
     * Gets the motion vector from the joystick values.
     * @param leftStickX The left joystick X.
     * @param leftStickY The left joystick Y.
     * @param rightStickX The right joystick X.
     * @param rightStickY The right joystick Y.
     * @return The Mecanum motion vector.
     */
    public static Motion joystickToMotion(double leftStickX,
                                          double leftStickY,
                                          double rightStickX,
                                          double rightStickY) {
        double vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) +
                        Math.pow(leftStickY, 2)), 1)*1.5;
        double thetaD = Math.atan2(-leftStickX, -leftStickY);
        double vTheta = -rightStickX;
        return new Motion(vD, thetaD, vTheta);
    }

    /**
     * Mecanum wheels, used to get individual motor powers.
     */
    public static class Wheels {
        // The mecanum wheels.
        public static double frontLeft;// ALL OF THESE VARIABLES WERE NON STATIC AT ONE POINT
        public static double frontRight;
        public static double backLeft;
        public static double backRight;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                    backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        /**
         * Scales the wheel powers by the given factor.
         * @param scalar The wheel power scaling factor.
         */
        public Wheels scaleWheelPower(double scalar) {
            return new Wheels(frontLeft * scalar, frontRight * scalar,
                    backLeft * scalar, backRight * scalar);
        }

        public static Wheels scaleWheelPower(){// THIS WAS NOT STATIC AT ONE POINT
                frontLeft = frontLeft * 100;
                frontRight = frontRight * 100;
                backLeft = backLeft * 100;
                backRight = backRight * 100;
            if (frontLeft>=0){
                frontLeft=(float)1.2*Math.pow(1.043,frontLeft)-1.2+(0.2*frontLeft);
            }
            else{
                frontLeft=-frontLeft;
                frontLeft=(float)1.2*Math.pow(1.043,frontLeft)-1.2+(0.2*frontLeft);
                frontLeft=-frontLeft;
            }
            if(frontRight>=0){
                frontRight=(float)1.2*Math.pow(1.043,frontRight)-1.2+(0.2*frontRight);
            }
            else{
                frontRight=-frontRight;
                frontRight=(float)1.2*Math.pow(1.043,frontRight)-1.2+(0.2*frontRight);
                frontRight=-frontRight;
            }
            if(backLeft>=0){
                backLeft=(float)1.2*Math.pow(1.043,backLeft)-1.2+(0.2*backLeft);
            }
            else{
                backLeft=-backLeft;
                backLeft=(float)1.2*Math.pow(1.043,backLeft)-1.2+(0.2*backLeft);
                backLeft=-backLeft;
            }
            if(backRight>=0){
                backRight=(float)1.2*Math.pow(1.043,backRight)-1.2+(0.2*backRight);
            }
            else{
                backRight=-backRight;
                backRight=(float)1.2*Math.pow(1.043,backRight)-1.2+(0.2*backRight);
                backRight=-backRight;
            }
            return new Wheels(frontLeft/100.0f, frontRight/100.0f, backLeft/100.0f, backRight/100.0f);
        }
        public static Wheels scaleWheelPowerTwo(){
            frontLeft=frontLeft/2.0f;
            frontRight=frontRight/2.0f;
            backLeft=backLeft/2.0f;
            backRight=backRight/2.0f;
            return new Wheels(frontLeft, frontRight, backLeft, backRight);
        }

    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheels(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
        double frontRight  = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
        double backLeft = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;
        double backRight = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
        return new Wheels(frontLeft, frontRight,
                backLeft, backRight);
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
        double minPower = Collections.min(powers);
        double maxPower = Collections.max(powers);
        double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

        if (maxMag > 1.0) {
            for (int i = 0; i < powers.size(); i++) {
                powers.set(i, powers.get(i) / maxMag);
            }
        }
    }
}
