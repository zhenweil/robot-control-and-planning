#include "object_placement_planning/cmaes.hpp"

#include <algorithm>
#include <cmath>

namespace cmaes
{

Cmaes::Cmaes(const Eigen::VectorXd& initial_mean, double initial_step_size, unsigned int random_seed, int population_size)
	: dimension(static_cast<int>(initial_mean.size())), mean(initial_mean), step_size(initial_step_size), random_engine(random_seed)
{
	const double n = static_cast<double>(this->dimension);
	this->population_size = population_size > 0 ? population_size : 4 + static_cast<int>(std::floor(3.0 * std::log(n)));
	this->num_parents = this->population_size / 2;
	this->recombination_weights.resize(this->num_parents);
	for (int parent_index = 0; parent_index < this->num_parents; ++parent_index)
		this->recombination_weights(parent_index) = std::log(0.5 * (this->population_size + 1)) - std::log(parent_index + 1.0);
	this->recombination_weights /= this->recombination_weights.sum();
	this->effective_num_parents = 1.0 / this->recombination_weights.squaredNorm();
	const double mu_eff = this->effective_num_parents;
	this->covariance_path_rate = (4.0 + mu_eff / n) / (n + 4.0 + 2.0 * mu_eff / n);
	this->step_size_path_rate = (mu_eff + 2.0) / (n + mu_eff + 5.0);
	this->rank_one_rate = 2.0 / ((n + 1.3) * (n + 1.3) + mu_eff);
	this->rank_parents_rate = std::min(1.0 - this->rank_one_rate, 2.0 * (mu_eff - 2.0 + 1.0 / mu_eff) / ((n + 2.0) * (n + 2.0) + mu_eff));
	this->step_size_damping = 1.0 + 2.0 * std::max(0.0, std::sqrt((mu_eff - 1.0) / (n + 1.0)) - 1.0) + this->step_size_path_rate;
	this->expected_normal_norm = std::sqrt(n) * (1.0 - 1.0 / (4.0 * n) + 1.0 / (21.0 * n * n));
	this->covariance = Eigen::MatrixXd::Identity(this->dimension, this->dimension);
	this->covariance_path = Eigen::VectorXd::Zero(this->dimension);
	this->step_size_path = Eigen::VectorXd::Zero(this->dimension);
	this->updateDecomposition();
}

std::vector<Eigen::VectorXd> Cmaes::askGeneration()
{
	std::vector<Eigen::VectorXd> points(static_cast<size_t>(this->population_size));
	for (Eigen::VectorXd& point : points)
	{
		Eigen::VectorXd standard_normal(this->dimension);
		for (int index = 0; index < this->dimension; ++index)
			standard_normal(index) = this->normal(this->random_engine);
		point = this->mean + this->step_size * this->covariance_sqrt * standard_normal;
	}
	return points;
}

void Cmaes::tellRankedGeneration(const std::vector<Eigen::VectorXd>& ranked_points)
{
	++this->num_generations;
	const double n = static_cast<double>(this->dimension);
	const Eigen::VectorXd old_mean = this->mean;
	this->mean = Eigen::VectorXd::Zero(this->dimension);
	for (int parent_index = 0; parent_index < this->num_parents; ++parent_index)
		this->mean += this->recombination_weights(parent_index) * ranked_points[static_cast<size_t>(parent_index)];
	const Eigen::VectorXd mean_shift = (this->mean - old_mean) / this->step_size;
	const double mu_eff = this->effective_num_parents;
	this->step_size_path = (1.0 - this->step_size_path_rate) * this->step_size_path + std::sqrt(this->step_size_path_rate * (2.0 - this->step_size_path_rate) * mu_eff) * this->covariance_inverse_sqrt * mean_shift;
	const double path_norm_scale = this->step_size_path.norm() / std::sqrt(1.0 - std::pow(1.0 - this->step_size_path_rate, 2.0 * this->num_generations));
	const bool path_not_stalled = path_norm_scale / this->expected_normal_norm < 1.4 + 2.0 / (n + 1.0);
	this->covariance_path = (1.0 - this->covariance_path_rate) * this->covariance_path + (path_not_stalled ? std::sqrt(this->covariance_path_rate * (2.0 - this->covariance_path_rate) * mu_eff) : 0.0) * mean_shift;
	Eigen::MatrixXd rank_parents_update = Eigen::MatrixXd::Zero(this->dimension, this->dimension);
	for (int parent_index = 0; parent_index < this->num_parents; ++parent_index)
	{
		const Eigen::VectorXd step = (ranked_points[static_cast<size_t>(parent_index)] - old_mean) / this->step_size;
		rank_parents_update += this->recombination_weights(parent_index) * step * step.transpose();
	}
	const double stall_correction = path_not_stalled ? 0.0 : this->covariance_path_rate * (2.0 - this->covariance_path_rate);
	this->covariance = (1.0 - this->rank_one_rate - this->rank_parents_rate) * this->covariance + this->rank_one_rate * (this->covariance_path * this->covariance_path.transpose() + stall_correction * this->covariance) + this->rank_parents_rate * rank_parents_update;
	this->step_size *= std::exp((this->step_size_path_rate / this->step_size_damping) * (this->step_size_path.norm() / this->expected_normal_norm - 1.0));
	this->updateDecomposition();
}

int Cmaes::getPopulationSize() const
{
	return this->population_size;
}

const Eigen::VectorXd& Cmaes::getMean() const
{
	return this->mean;
}

double Cmaes::getStepSize() const
{
	return this->step_size;
}

void Cmaes::updateDecomposition()
{
	this->covariance = 0.5 * (this->covariance + this->covariance.transpose());
	const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(this->covariance);
	const Eigen::VectorXd standard_deviations = solver.eigenvalues().cwiseMax(1e-20).cwiseSqrt();
	this->covariance_sqrt = solver.eigenvectors() * standard_deviations.asDiagonal();
	this->covariance_inverse_sqrt = solver.eigenvectors() * standard_deviations.cwiseInverse().asDiagonal() * solver.eigenvectors().transpose();
}

}  // namespace cmaes
