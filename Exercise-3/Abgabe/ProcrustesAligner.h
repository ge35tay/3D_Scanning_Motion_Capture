#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block<3,3>(0, 0) =  rotation;
		estimatedPose.block<3,1>(0, 3) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		int length = points.size();
		Vector3f mean = Vector3f::Zero();

		for (std::vector<Vector3f>::const_iterator iter = points.begin(); iter!= points.end(); iter++)
		{
			mean += *iter;
		}
		mean = mean / length;

		std::cout << "mean of the input points: [x, y, z]" << mean[0] << mean[1] << mean[2] << std::endl;
		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).

		Matrix3f rotation = Matrix3f::Identity(); 
		Matrix3f cross_covariance_matrix = Matrix3f::Zero();
		for (int i = 0; i < sourcePoints.size(); i++)
		{
			cross_covariance_matrix += (targetPoints[i] - targetMean)*(sourcePoints[i] - sourceMean).transpose();
		}

		// ComputeFullU: in JacobiSVD to indicate that the square matrix U is to be computed.
		// ComputeThinU：in JacobiSVD to indicate that the thin matrix U is to be computed.
		Eigen::JacobiSVD<Matrix3f> svd(cross_covariance_matrix, ComputeFullU | ComputeFullV);
		rotation = svd.matrixU() * svd.matrixV().transpose();
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.

		Vector3f translation = Vector3f::Zero();
		translation = - rotation * sourceMean + targetMean;
        return translation;
	}
};
