// Copyright (c) Choreo contributors

package choreo.trajectory;

import net.unnamedrobotics.lib.math.Pose;

import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * The generic interface for a sample in a trajectory.
 *
 * @param <Self> Derived sample type.
 */
public interface TrajectorySample<Self extends TrajectorySample<Self>>
    extends Interpolatable<Self> {
  /**
   * Returns the timestamp of this sample.
   *
   * @return the timestamp of this sample.
   */
  double getTimestamp();

  /**
   * Returns the pose at this sample.
   *
   * @return the pose at this sample.
   */
  Pose getPose();

  /**
   * Returns the field-relative chassis speeds of this sample.
   *
   * @return the field-relative chassis speeds of this sample.
   */
  Pose getChassisSpeeds();

  /**
   * Returns this sample, mirrored across the field midline.
   *
   * @return this sample, mirrored across the field midline.
   */
  Self flipped();

  /**
   * Returns this sample, offset by the given timestamp.
   *
   * @param timestampOffset the offset to apply to the timestamp.
   * @return this sample, offset by the given timestamp.
   */
  Self offsetBy(double timestampOffset);

  /**
   * For internal use only.
   *
   * @param length the length of the array to create.
   * @return the created array.
   */
  Self[] makeArray(int length);
}
