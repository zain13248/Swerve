package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private final double kAbsoluteEncoderOffset;
    private final boolean kAbsoluteEncoderReversed;



    private CANcoder absoluteEncoder ;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());



    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        kAbsoluteEncoderOffset = absoluteEncoderOffset;
        kAbsoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

  public SwerveModulePosition getPosition(){
    return( new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad())));
    }
    
    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        double desiredTurnSpeed = turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
        turningMotor.set(desiredTurnSpeed);
        driveMotor.set(state.speedMetersPerSecond);

        desiredState = state;
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }    

    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        turningEncoder.setPosition(this.getAbsoluteEncoderRotations());
    }
    
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }


    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // public double getAbsoluteEncoderRad() {
    //     // Get the absolute position of the encoder in degrees.
    //     double absolutePositionDegrees = this.absoluteEncoder.getAbsolutePosition().getValue();
    
    //     // Convert degrees to radians
    //     double radians = Math.toRadians(absolutePositionDegrees*360);
    
    
    //     return radians;
    // }
    
    public double getAbsoluteEncoderRotations() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Returns percent of a full rotation
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Returns percent of a full rotation
        angle = Units.rotationsToRadians(angle);
        return angle * (kAbsoluteEncoderReversed ? -1.0 : 1.0); // Look up ternary or conditional operators in java
    }
    

    


    // public void setDesiredState(SwerveModuleState state) {
    //     if (Math.abs(state.speedMetersPerSecond) < 0.001) {
    //         stop();
    //         return;
    //     }
    //     state = SwerveModuleState.optimize(state, getState().angle);
    //     driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //     turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    // }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }


}