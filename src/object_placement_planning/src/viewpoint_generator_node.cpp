// Generates viewpoints with no robot reachability check (viewpoint_generation.hpp) and writes
// selected_robot_poses.json / selected_viewpoints.json to output_dir, for object_placement's tour_input_dir.

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "object_placement_planning/viewpoint_generation.hpp"
#include "object_placement_planning/viewpoint_io.hpp"

class ViewpointGeneratorNode : public rclcpp::Node
{
public:
	ViewpointGeneratorNode() : Node("viewpoint_generator", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}

	void Run()
	{
		viewpoint_generation::Settings settings;
		settings.mesh_path = this->GetParameter<std::string>("mesh_path", "");
		if (settings.mesh_path.empty())
			settings.mesh_path = ament_index_cpp::get_package_share_directory("object_placement_planning") + "/meshes/bunny_holding_eggs.stl";
		settings.mesh_scale = this->GetParameter<double>("mesh_scale", settings.mesh_scale);
		settings.target_faces = static_cast<int>(this->GetParameter<int64_t>("target_faces", settings.target_faces));
		settings.candidate_generation_method = this->GetParameter<std::string>("candidate_generation_method", settings.candidate_generation_method);
		settings.n_surface_samples = static_cast<int>(this->GetParameter<int64_t>("n_surface_samples", settings.n_surface_samples));
		settings.standoff_distances = this->GetParameter<std::vector<double>>("standoff_distances", settings.standoff_distances);
		settings.tilt_angles_deg = this->GetParameter<std::vector<double>>("tilt_angles_deg", settings.tilt_angles_deg);
		settings.min_clearance = this->GetParameter<double>("min_clearance", settings.min_clearance);
		settings.fov_deg = this->GetParameter<double>("fov_deg", settings.fov_deg);
		settings.max_distance = this->GetParameter<double>("max_distance", settings.max_distance);
		settings.angle_threshold_deg = this->GetParameter<double>("angle_threshold_deg", settings.angle_threshold_deg);
		settings.max_rays_per_view = static_cast<int>(this->GetParameter<int64_t>("max_rays_per_view", settings.max_rays_per_view));
		settings.target_area_visibility = this->GetParameter<double>("target_area_visibility", settings.target_area_visibility);
		settings.random_seed = static_cast<int>(this->GetParameter<int64_t>("random_seed", settings.random_seed));
		settings.object_translation_world = ToVector3(this->GetParameter<std::vector<double>>("object_translation_world", {0.5, 0.0, 0.15}), settings.object_translation_world);
		settings.object_rotation_world = RotationFromRpyDeg(this->GetParameter<std::vector<double>>("object_rotation_rpy_deg", {0.0, 0.0, 0.0}));
		settings.t_tcp_camera = IsometryFromXyzQuat(this->GetParameter<std::vector<double>>("t_tcp_camera_xyz", {0.0, 0.0, 0.0}), this->GetParameter<std::vector<double>>("t_tcp_camera_quat_xyzw", {0.0, 0.0, 0.0, 1.0}));
		const std::string output_dir = this->GetParameter<std::string>("output_dir", "/tmp/viewpoints_unchecked");

		const viewpoint_generation::GeneratedViewpoints viewpoints = viewpoint_generation::GenerateViewpoints(settings);
		RCLCPP_INFO(this->get_logger(), "candidates %d, visible %d, selected %zu covering %.1f%% of the area (no reachability check)", viewpoints.num_candidates, viewpoints.num_visible_candidates, viewpoints.selected.size(), 100.0 * viewpoints.area_visibility);

		std::vector<const ViewpointCandidate*> selected;
		for (const ViewpointCandidate& viewpoint : viewpoints.selected)
			selected.push_back(&viewpoint);
		std::filesystem::create_directories(output_dir);
		ExportSelectedViewpoints(output_dir, selected);
		ExportSelectedRobotPoses(output_dir, selected);
		RCLCPP_INFO(this->get_logger(), "wrote %s/selected_robot_poses.json", output_dir.c_str());
	}

private:
	template <typename T>
	T GetParameter(const std::string& name, const T& fallback)
	{
		if (!this->has_parameter(name))
			this->declare_parameter(name, fallback);
		return this->get_parameter(name).get_value<T>();
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	const std::shared_ptr<ViewpointGeneratorNode> node = std::make_shared<ViewpointGeneratorNode>();
	node->Run();
	rclcpp::shutdown();
	return 0;
}
