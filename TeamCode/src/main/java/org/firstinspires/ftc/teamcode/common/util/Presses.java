package org.firstinspires.ftc.teamcode.common.util;

import java.util.*;

/**
 * Presses - Utility class for reliable button handling and toggling
 * Supports single presses, releases, toggles, and combo inputs.
 */
public class Presses {

    private boolean toggledVariable = false;
    private boolean wasPressedVariable = false;
    private ToggleGroup toggleGroup = null;

    // Constructor for individual Presses
    public Presses() {}

    // Constructor for group-managed Presses
    public Presses(ToggleGroup toggleGroup) {
        this.toggleGroup = toggleGroup;
        this.toggleGroup.addPress(this);
    }

    /** Returns true exactly once after a button is pressed */
    public boolean pressed(boolean input) {
        boolean result = !wasPressedVariable && input;
        wasPressedVariable = input;
        return result;
    }

    /** Returns true exactly once after a button is released */
    public boolean released(boolean input) {
        boolean result = wasPressedVariable && !input;
        wasPressedVariable = input;
        return result;
    }

    /** Returns true whenever button state changes (press or release) */
    public boolean change(boolean input) {
        boolean result = wasPressedVariable != input;
        wasPressedVariable = input;
        return result;
    }

    /** Sets the toggle state manually */
    public void setToggleFalse() { toggledVariable = false; }
    public void setToggleTrue() { toggledVariable = true; }

    /** Toggle logic (works both standalone or in a ToggleGroup) */
    public boolean toggle(boolean input) {
        if (pressed(input)) {
            if (toggleGroup != null) toggleGroup.toggle(this);
            else toggledVariable = !toggledVariable;
        }
        return toggledVariable;
    }

    /** Returns the current toggle state */
    public boolean returnToggleState() { return toggledVariable; }

    // =========================
    // GROUP HANDLING
    // =========================
    public static class ToggleGroup {
        private final List<Presses> pressesList = new ArrayList<>();

        /** Add a Presses instance to this toggle group */
        public void addPress(Presses presses) {
            pressesList.add(presses);
        }

        /** Activates one Presses instance, deactivates others */
        public void toggle(Presses pressedInstance) {
            for (Presses p : pressesList) {
                p.toggledVariable = (p == pressedInstance);
            }
        }

        /** Untoggles all presses in the group */
        public void untoggleAll() {
            for (Presses p : pressesList) p.toggledVariable = false;
        }
    }

    // =========================
    // COMBO HANDLING
    // =========================

    /** Returns true when all provided buttons are currently pressed */
    public static boolean comboPressed(boolean... inputs) {
        for (boolean in : inputs) if (!in) return false;
        return true;
    }

    // Map to track combo state per unique combo hash
    private static final Map<Integer, Boolean> comboPressStates = new HashMap<>();

    /**
     * Handles a toggle activated by pressing a combination (e.g. Options + Share)
     * Each unique combo has its own memory.
     */
    public static boolean comboToggle(boolean currentState, boolean... inputs) {
        int comboId = inputs.length;
        boolean comboDown = true;
	    for (boolean in : inputs)
            if (!in) {
                comboDown = false;
                break;
            }

    	boolean wasDown = Boolean.TRUE.equals(comboPressStates.getOrDefault(comboId, false));
    	boolean toggled = currentState;

        if (comboDown && !wasDown) toggled = !currentState; // toggle only on new combo press

        comboPressStates.put(comboId, comboDown);
        return toggled;
    }
}