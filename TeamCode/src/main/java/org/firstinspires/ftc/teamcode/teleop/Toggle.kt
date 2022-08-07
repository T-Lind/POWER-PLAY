package org.firstinspires.ftc.teamcode.teleop

/**
 * Toggle Object - code to detect button presses and update a toggle variable in a fast loop
 */
@Strictfp
class Toggle(
    /**
     * The state (true or false) that this object is set to
     */
    @field:Transient var toggleState: Boolean
) {
    /**
     * Get the value of the toggle
     * @return the value of the toggle
     * Precondition:  the object's starting conditions have been assigned correctly
     * Postcondition: the appropriate state is returned
     */
    /**
     * The past input that was provided to this object
     */
    @Transient
    private var pastInputState: Boolean

    /**
     * updates the current state according to a leading edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     * Precondition:  currentInputState accurately reflects the toggle state
     * Postcondition: the object's state has been updated according to a leading edge detector
     */
    fun updateLeadingEdge(currentInputState: Boolean) {
        // if the past input state was false and the current input state is true then
        // otherwise the current toggle state is false
        if (!pastInputState && currentInputState) // if the past toggle state was false then the current toggle state is true
            toggleState = !toggleState

        // set the past state to the last read state
        pastInputState = currentInputState
    }

    /**
     * updates the current state according to a falling edge detector
     * @param currentInputState what the input currently reads (ex. a button on the control pad)
     * Precondition:  currentInputState accurately reflects the toggle state
     * Postcondition: the object's state has been updated according to a falling edge detector
     */
    fun updateFallingEdge(currentInputState: Boolean) {
        // if the past input state was false and the current input state is true then
        // otherwise the current toggle state is false
        if (pastInputState && !currentInputState) // if the past toggle state was false then the current toggle state is true
            toggleState = !toggleState

        // set the past state to the last read state
        pastInputState = currentInputState
    }

    /**
     * Writes the toggle state to a variable
     * Additionally the past state is as a default set to startingToggleState.
     * @param startingToggleState the state the toggle should initially start in
     */
    init {
        pastInputState = toggleState
    }
}