package frc.robot.controls;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intakes {
    private final EaseofLife MotorMode = new EaseofLife();
    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }
    private DropState dropState = DropState.IDLE;

    // stay true once triggered, only reset when moving the opposite direction
    private boolean latchedUp   = true;  // assume starting position is UP
    private boolean latchedDown = false;

    public Intakes(TalonFX intakeMotor, TalonFX dropMotor) {
        this.intakeMotor = intakeMotor;
        this.dropMotor = dropMotor;
        MotorMode.setBrake(dropMotor, true); // assume starting position is UP
    }

    // keeps moving down until lower sensor trips
    public void startMovingDown(DigitalInput lowerSensor) {
        latchedUp = false; // leaving the top, clear the upper latch
        boolean isDown = !lowerSensor.get() || latchedDown;
        if (isDown) {
            System.out.println("IntakeDrop: Already at down position.");
            dropState = DropState.IDLE;
            return;
        }
        MotorMode.setBrake(dropMotor, false);
        MotorMode.setSpeed(dropMotor, -0.1); // TODO: tune speed
        dropState = DropState.MOVING_DOWN;
        System.out.println("IntakeDrop: Moving down");
    }

    // always returns arm upward regardless of current state
    public void startMovingUp(DigitalInput upperSensor) {
        latchedDown = false; // leaving the bottom, clear the lower latch
        boolean isUp = !upperSensor.get() || latchedUp;
        if (isUp) {
            System.out.println("IntakeDrop: Already at upper position.");
            MotorMode.setSpeed(dropMotor, 0);
            MotorMode.setBrake(dropMotor, true);
            dropState = DropState.IDLE;
            return;
        }
        MotorMode.setBrake(dropMotor, false);
        MotorMode.setSpeed(dropMotor, 0.1); // TODO: tune speed
        dropState = DropState.MOVING_UP;
        System.out.println("IntakeDrop: Moving UP");
    }

    // handles auto-stop on sensor hit
    public void periodic(DigitalInput upperSensor, DigitalInput lowerSensor) {
        // Update latches — once true, stays true until the opposite movement clears it
        if (!lowerSensor.get()) latchedDown = true;
        if (!upperSensor.get()) latchedUp   = true;

        if (dropState == DropState.MOVING_DOWN && latchedDown) {
            // stop but no brake
            MotorMode.setSpeed(dropMotor, 0);
            dropState = DropState.IDLE;
            System.out.println("IntakeDrop: Reached DOWN, motor stopped (no brake).");
        } else if (dropState == DropState.MOVING_UP && latchedUp) {
            // stop and brake to hold position
            MotorMode.setSpeed(dropMotor, 0);
            MotorMode.setBrake(dropMotor, true);
            dropState = DropState.IDLE;
            System.out.println("IntakeDrop: Reached UP, brake ON.");
        }
    }

    // Hold to spin intake, release to stop
    public void runIntake(int percent) {
        MotorMode.setSpeed(intakeMotor, percent);
    }

    public void stopIntake() {
        MotorMode.setSpeed(intakeMotor, 0);
    }
}