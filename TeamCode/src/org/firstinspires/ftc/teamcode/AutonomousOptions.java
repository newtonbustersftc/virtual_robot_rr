package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Emily 11-6-21 09:00-11:00
 * Updated the current year specifications
 */
@TeleOp(name="Autonomous Options", group="Main")

public class AutonomousOptions extends OpMode {

    // ADD preference names here
    public static final String START_POS_MODES_PREF = "starting position";
    public static final String START_DELAY_PREF = "start_delay";
    public static final String PARKING_PREF = "parking loc";
    public static final String[] START_POS_MODES = {"RED_LEFT", "RED_RIGHT","BLUE_LEFT", "BLUE_RIGHT"};
    public static final String[] START_DELAY = {"0 sec", "1 sec", "2 sec", "3 sec", "4 sec", "5 sec"};
    public static final String[] PARKING_LOCATION = {"CORNER", "WALL"};

    public static Map<String, String[]> prefMap = new HashMap<>();
    private static String[] prefKeys = {START_POS_MODES_PREF, START_DELAY_PREF, PARKING_PREF};
    private static String NONE = "";
    private static int keyIdx = 0;

    //private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);

    static {
        // ADD entries to preference map here
        prefMap.put(START_DELAY_PREF, START_DELAY);
        prefMap.put(START_POS_MODES_PREF, START_POS_MODES);
        prefMap.put(PARKING_PREF, PARKING_LOCATION);
    }

    public boolean isUpPressed;
    public boolean isDownPressed;
    public boolean isRightPressed;
    public boolean isLeftPressed;

    private int selectionIdx = 0;


    private int getIndex(String val, String[] array) {
        if (array!=null) {
            for (int i = 0; i < array.length; i++) {
                if (array[i].equals(val)) {
                    return i;
                }
            }
        }
        return -1;
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        displayAll();
    }

    private void displayAll () {
        telemetry.clear();

    }
    void updateAutoPref(String key, String value) {

    }
}


