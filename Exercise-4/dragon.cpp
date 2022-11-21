#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function (check gaussian.cpp for reference)
struct RegistrationCostFunction
{
		RegistrationCostFunction(const Point2D& point1_, const Point2D& point2_, const Weight& weight_)
		: point1(point1_), point2(point2_), weight(weight_)
	{
	}

	template<typename T>
	bool operator()(const T* const tx, const T* const ty, const T* const angle, T* residual) const
	{
		// TODO: Implement the cost function
		T loss_x = (cos(angle[0]) * point1.x - sin(angle[0]) * point1.y + tx[0])- point2.x;
		T loss_y = (sin(angle[0]) * point1.x + cos(angle[0]) * point1.y + ty[0])- point2.y;
		residual[0] = weight.w * (pow(loss_x, 2) + pow(loss_y, 2));
		return true;
	}

private:
	const Point2D point1;
	const Point2D point2;
	const Weight weight;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../../Data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../../Data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../../Data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block (check gaussian.cpp for reference)
	int len = points1.size();
	for (int i=0; i < len; i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 1, 1, 1, 1>(
				new RegistrationCostFunction(points1[i], points2[i], weights[i])
			),
			nullptr, &tx, &ty, &angle
		);
	}
	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
