#pragma once
// CMA-ES (Hansen, "The CMA Evolution Strategy: A Tutorial", 2016), plain (mu/mu_w, lambda) with the default settings, for
// small searches such as the object position. Ask for a generation, score it, tell the points back best first.

#include <random>
#include <vector>

#include <Eigen/Dense>

namespace cmaes
{

class Cmaes
{
public:
	// population_size 0 = the default 4 + floor(3 ln n).
	Cmaes(const Eigen::VectorXd& initial_mean, double initial_step_size, unsigned int random_seed, int population_size = 0);

	// The next generation's points.
	std::vector<Eigen::VectorXd> askGeneration();
	// The same points ranked best first (the search only needs the ranking); updates mean, step size and covariance.
	void tellRankedGeneration(const std::vector<Eigen::VectorXd>& ranked_points);

	int getPopulationSize() const;
	const Eigen::VectorXd& getMean() const;
	double getStepSize() const;

private:
	int dimension = 0;
	int population_size = 0;
	int num_parents = 0;
	Eigen::VectorXd recombination_weights;
	double effective_num_parents = 0.0;
	double covariance_path_rate = 0.0;
	double step_size_path_rate = 0.0;
	double rank_one_rate = 0.0;
	double rank_parents_rate = 0.0;
	double step_size_damping = 0.0;
	double expected_normal_norm = 0.0;

	Eigen::VectorXd mean;
	double step_size = 0.0;
	Eigen::MatrixXd covariance;
	Eigen::MatrixXd covariance_sqrt;  // B D: covariance = (B D)(B D)^T
	Eigen::MatrixXd covariance_inverse_sqrt;  // B D^-1 B^T
	Eigen::VectorXd covariance_path;
	Eigen::VectorXd step_size_path;
	int num_generations = 0;
	std::mt19937 random_engine;
	std::normal_distribution<double> normal{0.0, 1.0};

	void updateDecomposition();
};

}  // namespace cmaes
