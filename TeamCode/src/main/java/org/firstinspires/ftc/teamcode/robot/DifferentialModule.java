package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DifferentialModule {
    ServoImplEx left, right;
    public final static double HALF = 0.4;
    private double offsetLeft = 0.0;
    private double offsetRight = 0.0;
    private double base, diffLeft, diffRight;

    public DifferentialModule(ServoImplEx left, ServoImplEx right) {
        this.left = left;
        this.right = right;
        diffLeft = 0;
        diffRight = 0;
    }

    public DifferentialModule(ServoImplEx left, ServoImplEx right, double offsetL, double offsetR) {
        this.left = left;
        this.right = right;
        diffLeft = 0;
        diffRight = 0;
        offsetLeft = offsetL;
        offsetRight = offsetR;
    }

    /** Sets servos' positions. Call after a method. */
    private void apply() {
        left.setPosition(base + offsetLeft + diffLeft);
        right.setPosition(base + offsetRight + diffRight);
    }

    /**
     * Sets differences of the left servo and the right servo from the base (pitch).
     * @param left Difference between left servo and base. Pass in <code>2</code> for no change.
     * @param right Difference between right servo and base. Pass in <code>2</code> for no
     *              change.
     */
    private void setDifferences(double left, double right) {
        if (left <= 1) this.diffLeft = left;
        if (right <= 1) this.diffRight = right;
        apply();
    }

    /**
     * Sets differences of the left servo and the right servo from the base (pitch).
     * Shorthand for {@link #setDifferences(double, double)} when both values are equal in
     * magnitude but opposite in direction.
     * @param value Difference between servo and base.
     */
    public final void setDifferences(double value) {setDifferences(value, -value);}

    /** Enables power on both arm servos. */
    public final void setPwmEnable() {
        right.setPwmEnable();
        left.setPwmEnable();
    }

    /** Disables power on both arm servos. */
    public final void setPwmDisable() {
        left.setPwmDisable();
        right.setPwmDisable();
    }

    /**
     * Gets the current base pitch. Note that this does not reflect the orientation of the
     * module or the actual position values of the servos.
     * @return Pitch of the scoring module.
     */
    public final double getPitch() {return this.base;}

    /**
     * Sets the pitch of the scoring set. This does not interfere with the orientation (roll)
     * of the scoring module.
     * @param target The target value, relative to the servos.
     */
    public final void setPitch(double target) {
        this.base = target;
        apply();
    }

    /**
     * Sets the pitch and the differences of the servos. A combination of
     * {@link #setDifferences(double, double) setDifferences()} and
     * {@link #setPitch(double) setPitch()}.
     * @param pitch The target pitch, relative to the servos.
     * @param diffLeft Difference between left servo and base. Pass in <code>2</code> for no
     *                 change.
     * @param diffRight Difference between right servo and base. Pass in <code>2</code> for no
     *                  change.
     */
    private void setValues(double pitch, double diffLeft, double diffRight) {
        if (diffLeft <= 1) this.diffLeft = diffLeft;
        if (diffRight <= 1) this.diffRight = diffRight;
        this.base = pitch;
        apply();
    }

    /** Sets the orientation of scoring to a specific angle.
     * @param target The target (in degrees) for the module to rotate to.
     */
    public final void setOrientation(double target) {
        double distance = HALF * target / 90;
        setDifferences(distance, -distance);
    }

    public final void setPosition(double pitch, double orientation) {
        double difference = HALF * orientation / 90;
        setValues(pitch, difference, -difference);
    }
}