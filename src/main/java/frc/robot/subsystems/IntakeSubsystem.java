package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax IntakeInOut, IntakeLeft, IntakeRight;

    public IntakeSubsystem(Integer IntakeInOut_ID, Integer IntakeLeft_ID, Integer IntakeRight_ID) {
        IntakeInOut = new CANSparkMax(IntakeInOut_ID, CANSparkLowLevel.MotorType.kBrushless);
        IntakeLeft = new CANSparkMax(IntakeLeft_ID, CANSparkLowLevel.MotorType.kBrushless);
        IntakeRight = new CANSparkMax(IntakeRight_ID, CANSparkLowLevel.MotorType.kBrushless);


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

    
}