package org.firstinspires.ftc.teamcode.mainModules; //place where the code is located

import java.util.ArrayList;
import java.util.List;

public class Presses {

    private boolean toggledVariable = false;
    private boolean wasPressedVariable = false;
    private ToggleGroup toggleGroup = null; // Reference to the group

    // Constructor for individual Presses (not part of a group)
    public Presses() {}

    // Constructor for group-managed Presses
    public Presses(ToggleGroup toggleGroup) {
        this.toggleGroup = toggleGroup;
        this.toggleGroup.addPress(this);
    }

    // Returns true after the button is released
    public boolean released(boolean inputBoolean) {
        if (wasPressedVariable != inputBoolean && wasPressedVariable) {
            wasPressedVariable = inputBoolean;
            return true;
        } else {
            wasPressedVariable = inputBoolean;
            return false;
        }
    }

    // Returns true after the button state is changed
    public boolean change(boolean inputBoolean) {
        if (wasPressedVariable != inputBoolean) {
            wasPressedVariable = inputBoolean;
            return true;
        } else {
            wasPressedVariable = inputBoolean;
            return false;
        }
    }

    // Returns true after the button is pressed
    public boolean pressed(boolean inputBoolean) {
        if (wasPressedVariable != inputBoolean && !wasPressedVariable) {
            wasPressedVariable = inputBoolean;
            return true;
        } else {
            wasPressedVariable = inputBoolean;
            return false;
        }
    }

    // Sets the toggled state to false
    public void setToggleFalse() {
        toggledVariable = false;
    }

    // Sets the toggled state to true
    public void setToggleTrue() {
        toggledVariable = true;
    }

    // Allows toggling of a button after presses
    public boolean toggle(boolean inputBoolean) {
        if (pressed(inputBoolean)) {
            if (toggledVariable) {
                toggledVariable = false;  // Untoggle if already toggled
            } else {
                if (toggleGroup != null) {
                    toggleGroup.toggle(this);  // Notify the group to toggle this and untoggle others
                } else {
                    toggledVariable = !toggledVariable;  // Individual toggle if not part of a group
                }
            }
        }
        return toggledVariable;
    }

    // For easily returning the toggled state
    public boolean returnToggleState() {
        return toggledVariable;
    }

    // Inner class to manage a group of Presses instances
    public static class ToggleGroup {
        private List<Presses> pressesList = new ArrayList<>();

        // Add a Presses instance to the group
        public void addPress(Presses presses) {
            pressesList.add(presses);
        }

        // Toggle a specific Presses instance and untoggle the others
        public void toggle(Presses pressedInstance) {
            for (Presses presses : pressesList) {
                if (presses == pressedInstance) {
                    presses.setToggleTrue();  // Toggle the selected one
                } else {
                    presses.setToggleFalse(); // Untoggle all others
                }
            }
        }

        // Untoggle all Presses in the group
        public void untoggleAll() {
            for (Presses presses : pressesList) {
                presses.setToggleFalse();
            }
        }
    }
}
