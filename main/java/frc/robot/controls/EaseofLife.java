package frc.robot.controls;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class EaseofLife {
    public void setSpeed(TalonFX motor, double percentage) {
        motor.set(percentage);
    }

    public void setVelocity() {

    }

    public void setBrake(TalonFX motor, boolean brake) {
        boolean isStopped = Math.abs(motor.get()) < 0.01;
        if (brake && !isStopped) {
            System.err.println("EaseofLife: Motor must be stopped before braking.");
            return;
        }
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public boolean BmagnetSensor(DigitalInput sensor) {
        return !sensor.get();
    }
}