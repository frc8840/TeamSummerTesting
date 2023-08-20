package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class CANCoderA {
    CANCoder m_turningEncoder = new CANCoder(23);
    private MotorController m_driveMotor = new CANSparkMax(11, MotorType.kBrushless);
    private MotorController m_turningMotor = new CANSparkMax(12, MotorType.kBrushless);
    private int pos = 0;


    public void move() {
        m_turningEncoder.setPosition(pos);
        pos += 100;
    }
}