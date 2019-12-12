package org.darbots.darbotsftclib.libcore.calculations.timecalculation;

import java.util.Calendar;
import java.util.Date;

public class TimeCalculation {
    public static long getCurrentMillisUTCTimeFromEPOCH(){
        Calendar d = Calendar.getInstance();
        return d.getTimeInMillis();
    }
    public static Calendar getCurrentTime(){
        return Calendar.getInstance();
    }
    public static Calendar getDateFromTimeStamp(long MillisUTCFromEPOCH){
        Calendar d = Calendar.getInstance();
        d.setTimeInMillis(MillisUTCFromEPOCH);
        return d;
    }
}
