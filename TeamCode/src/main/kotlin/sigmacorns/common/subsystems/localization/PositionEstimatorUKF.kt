/**
 * INCOMPLETE UKF PoseEstimator. Other than the missing numpy import, the following are lacking:
 * - (Minor) Cholesky decomposition implementation (seems to be O(N^3)?)
 * - (Major) Testing to approximate the observed sigma points by a Gaussian distribution
 * Also, the predict step should be reverted to the simple KF since odometry is linear.
 * The code is commented out since there are a lot of errors (for placeholders) which is annoying.
 */

package sigmacorns.common.subsystems.localization

class PositionEstimatorUKF {

//    var error = Pose()
//    var kalmanGain = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    var measure = Pose()
//    var measureCovar = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    val measureNoiseCovar = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    var pose = Pose()
//    var poseOld = pose
//    var poseCovar = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    var poseCovarOld = poseCovar
//    var poseCovarSqrt = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    var sigmaPts = arrayOf(Pose(), Pose(), Pose(), Pose(), Pose(), Pose(), Pose())
//    val transitionNoiseCovar = arrayOf(arrayOf(0, 0, 0), arrayOf(0, 0, 0), arrayOf(0, 0, 0))
//    val weight0 = 0.1
//    val weightElse = (1 - weight0) / 6
//    val weightSigma = Math.sqrt(3 / (1 - weight0))
//
//    fun update() {
//        poseCovarSqrt = Math.choleskyDecompose(poseCovarOld)
//        sigmaPts[0] = poseOld
//        for (i in IntRange(0, 1)) {
//            for (j in IntRange(0, 2)) {
//                sigmaPts[3 * i + j + 1] = poseOld + weightSigma * ((-1) ** i) * poseCovarSqrt[j]
//            }
//        }
//
//        pose = f(poseOld).times(weight0)
//        for (i in IntRange(1, 6)) {
//            pose += weightElse * f(sigmaPts[i]) // exp log
//        }
//
//        poseCovar = transitionNoiseCovar
//        for (i in IntRange(1, 6)) {
//            error = f(sigmaPts[i]) - pose
//            poseCovar += weightElse * error * Math.transpose(error)
//        }
//
//        poseCovarSqrt = Math.choleskyDecompose(poseCovar)
//        sigmaPts[0] = pose
//        for (i in IntRange(0, 1)) {
//            for (j in IntRange(0, 2)) {
//                sigmaPts[3 * i + j + 1] = pose + weightSigma * ((-1) ** i) * poseCovarSqrt[j]
//            }
//        }
//
//        measure = h(pose).times(weight0)
//        for (i in IntRange(1, 6)) {
//            measure += weightElse * h(sigmaPts[i]) // exp log
//        }
//
//        measureCovar = measureNoiseCovar
//        for (i in IntRange(1, 6)) {
//            error = h(sigmaPts[i]) - measure
//            measureCovar += weightElse * error * Math.transpose(error)
//        }
//
//        kalmanGain = C_xz * measureCovar^(-1)
//
//    }
//
//    fun cholesky(): Array<Array<Double>> {
//        return arrayOf(arrayOf(0.0))
//    }

}