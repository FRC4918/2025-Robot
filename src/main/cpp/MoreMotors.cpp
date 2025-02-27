#include "MoreMotors.h"

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/Phoenix.h>

using namespace rev::spark;


/*---------------------------------------------------------------------*/
/* MotorInitSpark()                                                    */
/* Setup the initial configuration of a Neo motor, driven by a         */
/* Spark Max controller. Originally for elevator and sideshift motors. */
/* These settings can be superseded after this function is called,     */
/* for the needs of each specific SparkMax-driven motor.               */
/*---------------------------------------------------------------------*/
void ElevatorMotorInitSpark(SparkMax &m_motor)
{
    SparkMaxConfig config{};

    config.softLimit.ForwardSoftLimitEnabled(false);
    config.softLimit.ReverseSoftLimitEnabled(false);

    // Configure Spark Max Output direction.
    config.Inverted(false);
   

   /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                        // 10 Amps below 5000 RPM, above 5000 RPM it ramps from
                        // 10 Amps down to  5 Amps at 5700 RPM
    config.SmartCurrentLimit(2, 1, 5000);
#else
                        // 80 Amps below 5000 RPM, above 5000 RPM it ramps from
                        // 80 Amps down to 10 Amps at 5700 RPM
                        // We may have to try different CurrentLimits here to
                        // eliminate drivetrain chattering.
    config.SmartCurrentLimit(30, 10, 5000);
#endif
    config.limitSwitch.ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyOpen)
                        .ForwardLimitSwitchEnabled(false);

    // Config 100% motor output to 12.0V
    config.VoltageCompensation(12.0);

    // Set ramp rate (how fast motor accelerates or decelerates).
    // We may have to try different RampRates here to
    // eliminate drivetrain chattering.
    config.OpenLoopRampRate(0.2);
    config.ClosedLoopRampRate(0.2);

    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    m_motor.Configure(config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

} // MotorInitSpark()


void MotorInitVictor( WPI_VictorSPX &m_motor )
  {
    m_motor.ConfigFactoryDefault( 10 );
    m_motor.SetInverted( false );
                                             /* Set the peak and nominal outputs */
    m_motor.ConfigNominalOutputForward( 0, 10 );
    m_motor.ConfigNominalOutputReverse( 0, 10 );
    m_motor.ConfigPeakOutputForward(    1, 10 );
    m_motor.ConfigPeakOutputReverse(   -1, 10 );

            /* Set limits to how much current will be sent through the motor */
    //m_motor.ConfigPeakCurrentDuration(1);  // 1000 milliseconds (for 60 Amps)
#ifdef SAFETY_LIMITS
  //m_motor.ConfigPeakCurrentLimit(10);       // limit motor power severely
  //m_motor.ConfigContinuousCurrentLimit(10); // to 10 Amps
#else
    //m_motor.ConfigPeakCurrentLimit(60);          // 60 works here for miniCIMs,
                                                 // or maybe 40 Amps is enough,
                                                 // but we reduce to 10, 1, 10
    //m_motor.ConfigContinuousCurrentLimit(60);    // for safety while debugging
#endif
    //m_motor.EnableCurrentLimit(true);

                                          // Config 100% motor output to 12.0V
    m_motor.ConfigVoltageCompSaturation( 12.0 );
    m_motor.EnableVoltageCompensation( false );

    m_motor.SetNeutralMode( NeutralMode::Brake );

  }      // MotorInitVictor()