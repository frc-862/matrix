package frc.robot.lib.swervelib.ctre;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.lib.swervelib.AbsoluteEncoder;
import frc.robot.lib.swervelib.AbsoluteEncoderFactory;

public class ThriftyBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public ThriftyBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public ThriftyBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<ThriftyConfig> build() {
        return configuration -> {
            AnalogEncoder encoder = new AnalogEncoder(configuration.getId());
            // encoder.setPositionOffset(configuration.getOffset());
            encoder.setDistancePerRotation(1); //TODO: check

            return new EncoderImplementation(encoder, configuration.getOffset());
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final AnalogEncoder encoder;
        private final double offset;

        private EncoderImplementation(AnalogEncoder encoder, double offset) {
            this.encoder = encoder;
            this.offset = offset;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians((encoder.getAbsolutePosition() * 360) - offset);
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE, COUNTER_CLOCKWISE
    }
}
