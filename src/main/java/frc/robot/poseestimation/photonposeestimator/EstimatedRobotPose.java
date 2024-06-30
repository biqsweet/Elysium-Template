/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.poseestimation.photonposeestimator;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * An estimated pose based on pipeline result
 *
 * @param estimatedPose    The estimated pose
 * @param timestampSeconds The estimated time the frame used to derive the robot pose was taken
 * @param targetsUsed      A list of the targets used to compute this pose
 * @param strategy         The strategy actually used to produce this pose
 */
public record EstimatedRobotPose(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed,
                                 PhotonPoseEstimator.PoseStrategy strategy) {
    /**
     * Constructs an EstimatedRobotPose
     *
     * @param estimatedPose    estimated pose
     * @param timestampSeconds timestamp of the estimate
     */
    public EstimatedRobotPose {
    }
}
