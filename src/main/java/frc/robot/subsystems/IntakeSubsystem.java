package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkBase.ControlType;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase {
    public PivotState pivotState;
    private final SparkMax pivotMotor;
    private final SparkMax rollerMotor;

    private final SparkMaxConfig pivotConfig;
    private final SparkMaxConfig rollerConfig;

    private final RelativeEncoder pivotEncoder;
    private final PIDController pivotPIDController;
    
    @SuppressWarnings("removal")
    public IntakeSubsystem() {
        pivotMotor = new SparkMax(DriverConstants.PIVOT_INTAKE_MOTOR, MotorType.kBrushless);
        rollerMotor = new SparkMax(DriverConstants.ROLLER_INTAKE_MOTOR, MotorType.kBrushless);
        pivotConfig = new SparkMaxConfig();
        rollerConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        rollerConfig.idleMode(IdleMode.kBrake);
        pivotEncoder = pivotMotor.getEncoder();
        pivotPIDController = new PIDController(ModuleConstants.INTAKE_P, ModuleConstants.INTAKE_I, ModuleConstants.INTAKE_D);
        pivotMotor.configure(
            pivotConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
      }

    public void stop() {
        rollerMotor.set(0);
        pivotMotor.set(0);
    }

    public void stopPivot() {
      pivotMotor.set(0);
    }

    public void toggleRoller() {
      rollerMotor.set(rollerMotor.get()==0 ? ModuleConstants.INTAKE_CONSTANT : 0);
    }

    public void setPivotPos(double rot) {
      pivotMotor.set(pivotPIDController.calculate(pivotEncoder.getPosition(), rot));
    }

    public double getPivotDeg() {
      return Math.toDegrees(pivotEncoder.getPosition());
    }

  public Command IntakeMethodCommand() {return runOnce(() -> {});}

  @Override
  public void periodic() {
    double currentDeg = getPivotDeg();
    if (pivotState==PivotState.RAISED_RAISING) {
      setPivotPos(ModuleConstants.INTAKE_UPPER_RAISED);
      if (currentDeg<=ModuleConstants.INTAKE_UPPER_RAISED+5)
        pivotState = PivotState.RAISED_LOWERING;
    } else if (pivotState==PivotState.RAISED_LOWERING) {
      setPivotPos(ModuleConstants.INTAKE_LOWER_RAISED);
      if (currentDeg>=ModuleConstants.INTAKE_LOWER_RAISED+5)
        pivotState = PivotState.RAISED_RAISING;
    }
  }

  @Override
  public void simulationPeriodic() {}

  public enum PivotState {RAISED_RAISING, RAISED_LOWERING, LOWERING, LOWERED}
}
