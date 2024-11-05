package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax IntakeInOut, IntakeLever;
    private RelativeEncoder LeverEncoder;

    public IntakeSubsystem(Integer IntakeInOut_ID, Integer IntakeLever_ID) {
        IntakeInOut = new CANSparkMax(IntakeInOut_ID, CANSparkLowLevel.MotorType.kBrushless);
        IntakeLever = new CANSparkMax(IntakeLever_ID, CANSparkLowLevel.MotorType.kBrushless);

        LeverEncoder = IntakeLever.getEncoder();
    }
    
    public void intakeIn(){
        IntakeInOut.set(1);
    }

    public void intakeOut(){
        IntakeInOut.set(-1);
    }
    public void intakeStop(){
        IntakeInOut.set(0);
    }

    public void intakeLeverSpeed(double speed){
        IntakeLever.set(speed);
    }

    public boolean goToAngle(double lever, double limit, double kP, double threshold){
        double delta = Math.abs(lever) - Math.abs(LeverEncoder.getPosition());
        
        if(Math.abs(delta) >= threshold) {
            var motorSpeed = delta*kP;
            motorSpeed = Math.abs(motorSpeed) > limit ? limit * Math.signum(motorSpeed) : motorSpeed;
            intakeLeverSpeed(motorSpeed);
            return false;
        } else {
            intakeLeverSpeed(0);
            return true;
        }
    }



    
}