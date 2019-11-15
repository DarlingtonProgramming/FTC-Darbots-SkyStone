package org.darbots.darbotsftclib.libcore.templates.log;


public enum LogLevel{
    FATAL(5),
    ERROR(4),
    WARNING(3),
    INFO(2),
    DEBUG(1);

    private int value = 0;

    private LogLevel(int value) {    //    必须是private的，否则编译错误
        this.value = value;
    }

    public static LogLevel valueOf(int value) {
        switch (value) {
            case 1:
                return DEBUG;
            case 2:
                return INFO;
            case 3:
                return WARNING;
            case 4:
                return ERROR;
            case 5:
                return FATAL;
            default:
                return null;
        }
    }

    public int value() {
        return this.value;
    }
}