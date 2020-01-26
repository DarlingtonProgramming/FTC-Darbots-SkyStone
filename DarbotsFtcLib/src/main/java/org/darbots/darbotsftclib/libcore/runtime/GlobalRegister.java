package org.darbots.darbotsftclib.libcore.runtime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.integratedfunctions.logger.OpModeRunLog;

public class GlobalRegister {
    public static DarbotsBasicOpMode runningOpMode = null;
    public static OpModeRunLog currentLog = null;
}
