package frc.robot.Constants;

public class SubsystemConstants {

        public static class LightSensor{
            public static final int INTAKE_SENSOR_ID = 0;
            public static final int INDEXER_SENSOR_ID = 0;
            public static final int HOPPER_MAX_LIMIT_SENSOR_ID = 0;
        }

        public static class Vision{
            public static final String CAM_LIMELIGHT = "limeLight-knight";
                public static final double LL_X_MULTIPLIER = 1;
                public static final double LL_Y__MULTIPLIER = 1;
                public static final double LL_Z_MULTIPLIER = 1;

        }

        public static class ClimbSubsystemConstants{
            public static final int CLIMBMOTOR_A = 0;
            public static final int CLIMBMOTOR_B = 0;

            //SPEED CONSTANTS
                public static final double CLIMB_MANUAL_SPEED = 0;
                public static final double CLIMB_MAX_VELOCITY = 0;
                public static final double CLIMB_MAX_ACCELERATION = 0;
            //PID - Climb
                public static double CLIMB_kP = 0;
                public static double CLIMB_kI = 0;
                public static double CLIMB_kD = 0;
            //FEEDFORWARD - Climb
                public static double CLIMB_kS = 0;
                public static double CLIMB_kG = 0;
                public static double CLIMB_kV = 0;
                public static double Climb_kA = 0;

            //HARDWARE CONSTANTS
                public static double CLIMB_WHEEL_RADIUS_INCHES = 0;
            
            //CLIMB HEIGHT CONSTRAINTS
                public static double MAX_INCHES = 0;
                public static double MIN_INCHES = 0;
        
        }

        public static class IntakeSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 0;
            public static final int INTAKE_PIVOT_MOTOR_ID = 0;

            //HARDWARE CONSTANTS
                public static final double SLIDER_OFFSET = 0;
                public static final double ABS_ENC_2_INCHES_RATIO = 0;

            //SPEED CONSTANTS
                public static final double INTAKE_SPEED = 0;
                public static final double OUTTAKE_SPEED = 0;

            //PID - Slider
                public static double INTAKE_SLIDER_kP = 0;
                public static double INTAKE_SLIDER_kI = 0;
                public static double INTAKE_SLIDER_kD = 0;
            
            //Slider Macros
                public static double INTAKE_SLIDER_INCHES = 0;
                public static double OUTTAKE_SLIDER_INCHES = 0;
                public static double STOW_SLIDER_INCHES = 0;
        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 0;
            public static final int HOPPER_ID_B = 0;
            public static final int HOPPER_ID_C = 0;
            public static final int INDEX_MOTOR_ID = 0;
            
            //SPEED CONSTANTS
                public static final double INDEXER_SPEED = 0;
                public static final double HOPPER_SPEED = 0;

        }

        public static class ShooterSubsystemConstants{
            public static final int APRILTAG_TARGET_FIDUCIALID = 0;
            public static final int SHOOTER_ANGLE_ID = 0;
            public static final int SHOOTER_ID_A = 0;
            public static final int SHOOTER_ID_B = 0;
            public static final int SHOOTER_ID_C = 0;
            public static final int SHOOTER_ID_D = 0;
            //PID - Angle
                public static double SHOOTER_ANGLE_kP = 0;
                public static double SHOOTER_ANGLE_kI = 0;
                public static double SHOOTER_ANGLE_kD = 0;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0;
                public static double SHOOTER_SPEED_kI = 0;
                public static double SHOOTER_SPEED_kD = 0;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0;
                public static double SHOOTER_SPEED_kV = 0;
                public static double SHOOTER_SPEED_kA = 0;
            //HARDWARE CONSTANTS
                public static double FLYWHEEL2KRAKEN_GEAR_RATIO = 0.66666666666666666666666666666667; // 24 : 36
                public static double FLYWHEEL_RADIUS_METERS = 2;
                public static double SHOOTER_ANGLE_OFFSET = 0;
            //SHOOTER ANGLE CONSTRAINTS
                public static double MAX_ANGLE = 0;
                public static double MIN_ANGLE = 0;
                public static double SPEED_TOLERANCE = 0;
                public static double ANGLE_TOLERANCE = 0;
            //PERIODIC CONSTANTS
                public static boolean desiredVelReached = false;
                public static boolean desiredAngleReached = false;
        }
}