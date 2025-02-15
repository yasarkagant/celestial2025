package com.celestial.subsystems;

import com.celestial.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.EnumSet;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    private PIDController turningPidController;

    public SwerveModule(int driveMotorId, int rotationMotorId, boolean driveMotorReversed, boolean rotationMotorReversed, int absoluteEncoderChannel, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        driveMotor = new SparkMax(driveMotorId, SparkLowLevel.MotorType.kBrushless);
        rotationMotor = new SparkMax(rotationMotorId, SparkLowLevel.MotorType.kBrushless);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        DoubleTopic pTopic = inst.getDoubleTopic("swerveP");
        DoubleTopic dTopic = inst.getDoubleTopic("swerveD");

        DoubleSubscriber pSubscriber = pTopic.subscribe(0);
        DoubleSubscriber dSubscriber = dTopic.subscribe(0);

        pTopic.publish().set(0);
        dTopic.publish().set(0);

        turningPidController = new PIDController(Constants.ModuleConstants.kPRotation, 0, Constants.ModuleConstants.kDRotation);

        /*inst.addListener(
                pSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("Change");
                    turningPidController = new PIDController(pSubscriber.get(), 0, dSubscriber.get());
                });

        inst.addListener(
                dSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("Change");
                    turningPidController = new PIDController(pSubscriber.get(), 0, dSubscriber.get());
                });*/

        absoluteEncoder = new AnalogEncoder(absoluteEncoderChannel);
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        driveMotorConfig.inverted(driveMotorReversed);
        driveMotorConfig.encoder
                .positionConversionFactor(Constants.ModuleConstants.kDriveEncoderRot2Meter)
                .velocityConversionFactor(Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        SparkMaxConfig rotationMotorConfig = new SparkMaxConfig();
        rotationMotorConfig.inverted(rotationMotorReversed);
        rotationMotorConfig.encoder
                .positionConversionFactor(Constants.ModuleConstants.kRotationEncoderRot2Rad)
                .velocityConversionFactor(Constants.ModuleConstants.kRotationEncoderRPM2RadPerSec);

        driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        rotationMotor.configure(rotationMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }


    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getRotationPosition() {
        return rotationEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.get();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffset;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.04) {
            stop();
            return;
        }

        state.optimize(getState().angle);

        driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        rotationMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        SmartDashboard.putNumber("Target Angle", state.angle.getDegrees());
    }

    public void stop() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle, double currentVelocity) {
        double targetAngle = desiredState.angle.getRadians();
        double currentAngleRad = currentAngle.getRadians();

        // Normalize angles to [-π, π]
        double delta = Math.IEEEremainder(targetAngle - currentAngleRad, 2.0 * Math.PI);

        // Check if reversing the wheel direction is a better option
        if (Math.abs(delta) > Math.PI / 2) {
            targetAngle = Math.IEEEremainder(targetAngle + Math.PI, 2.0 * Math.PI);

            // Instead of instantly reversing at high speed, smoothly transition
            double reversedSpeed = -desiredState.speedMetersPerSecond;

            // If the current speed is high, apply a gradual transition
            if (Math.abs(currentVelocity) > 1.0) { // Threshold can be adjusted
                reversedSpeed *= 0.75; // Reduce speed slightly to prevent abrupt reversal
            }

            return new SwerveModuleState(reversedSpeed, new Rotation2d(targetAngle));
        }

        return new SwerveModuleState(desiredState.speedMetersPerSecond, new Rotation2d(targetAngle));
    }

}
