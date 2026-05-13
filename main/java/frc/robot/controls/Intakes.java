package frc.robot.controls;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intakes {
    private final EaseofLife MotorMode = new EaseofLife();
    private final TalonFX intakeMotor;
    private final TalonFX dropMotor;

    private enum DropState { IDLE, MOVING_DOWN, MOVING_UP }
    private DropState dropState = DropState.IDLE;
    private double moveStartTime;
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
        moveStartTime = Timer.getFPGATimestamp();
        latchedUp = false; // leaving the top, clear the upper latch
        boolean isDown = !lowerSensor.get() || latchedDown;
        if (isDown) {
            System.out.println("IntakeDrop: Already at down position.");
            dropState = DropState.IDLE;
            return;
        }
        if (!RobotBase.isSimulation()) MotorMode.setBrake(dropMotor, false);
        MotorMode.setSpeed(dropMotor, -0.1); // TODO: tune speed
        dropState = DropState.MOVING_DOWN;
        System.out.println("IntakeDrop: Moving down");
    }

    // always returns arm upward regardless of current state
    public void startMovingUp(DigitalInput upperSensor) {
        moveStartTime = Timer.getFPGATimestamp();
        latchedUp = !upperSensor.get(); // trust the sensor, not an assumption
        boolean isUp = !upperSensor.get() || latchedUp;
        if (isUp) {
            System.out.println("IntakeDrop: Already at upper position.");
            MotorMode.setSpeed(dropMotor, 0);
            if (!RobotBase.isSimulation()) MotorMode.setBrake(dropMotor, false);
            dropState = DropState.IDLE;
            return;
        }
        if (!RobotBase.isSimulation()) MotorMode.setBrake(dropMotor, true);
        MotorMode.setSpeed(dropMotor, 0.1); // TODO: tune speed
        dropState = DropState.MOVING_UP;
        System.out.println("IntakeDrop: Moving UP");
    }

    // handles auto-stop on sensor hit
    public void periodic(DigitalInput upperSensor, DigitalInput lowerSensor) {
        //once true, stays true until the opposite movement clears it
        if (!lowerSensor.get()) latchedDown = true;
        if (!upperSensor.get()) latchedUp   = true;
        if (dropState != DropState.IDLE && Timer.getFPGATimestamp() - moveStartTime > 2.0) { // 2 second timeout
            dropMotor.stopMotor();
            dropState = DropState.IDLE;
            DriverStation.reportWarning("IntakeDrop: Motion timeout!", false);
        }
        if (dropState == DropState.MOVING_DOWN && latchedDown) {
            // stop but no brake
            dropMotor.stopMotor();
            dropState = DropState.IDLE;
            System.out.println("IntakeDrop: Reached DOWN, motor stopped (no brake).");
        } else if (dropState == DropState.MOVING_UP && latchedUp) {
            // stop and brake to hold position
            dropMotor.stopMotor();
            MotorMode.setBrake(dropMotor, true);
            dropState = DropState.IDLE;
            System.out.println("IntakeDrop: Reached UP, brake ON.");
        }
    }

    // Hold to spin intake, release to stop
        public void runIntake(double percent) {
            MotorMode.setSpeed(intakeMotor, percent / 100.0);
        }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }
}