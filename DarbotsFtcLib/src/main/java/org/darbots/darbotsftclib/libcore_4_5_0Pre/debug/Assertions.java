package org.darbots.darbotsftclib.libcore_4_5_0Pre.debug;

public class Assertions {
    public static boolean isDoubleValid(double number){
        return !(Double.isNaN(number) || Double.isInfinite(number));
    }
}
