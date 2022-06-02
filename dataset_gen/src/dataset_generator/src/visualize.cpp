#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

std::vector<ignition::math::Pose3d> read_trajectory(std::string const& filename) {
    std::ifstream fin(filename);

    ignition::math::Pose3d point;
    std::vector<ignition::math::Pose3d> trajectory;
    while (fin >> point) {
        trajectory.push_back(point);
    }

    return trajectory;
}

void draw_axis(std::vector<ignition::math::Pose3d> const& trajectory) {
    ignition::transport::Node node;
    ignition::msgs::Marker x_marker, y_marker, z_marker;
    int id = 1;

    x_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    x_marker.set_type(ignition::msgs::Marker::LINE_LIST);
    x_marker.mutable_material()->mutable_script()->set_name("Gazebo/Red");
    x_marker.set_ns("axis");

    y_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    y_marker.set_type(ignition::msgs::Marker::LINE_LIST);
    y_marker.mutable_material()->mutable_script()->set_name("Gazebo/Green");
    y_marker.set_ns("axis");

    z_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    z_marker.set_type(ignition::msgs::Marker::LINE_LIST);
    z_marker.mutable_material()->mutable_script()->set_name("Gazebo/Blue");
    z_marker.set_ns("axis");

    for (auto& point : trajectory) {
        x_marker.set_id(id++);
        y_marker.set_id(id++);
        z_marker.set_id(id++);

        ignition::msgs::Set(x_marker.add_point(), point.Pos());
        ignition::msgs::Set(y_marker.add_point(), point.Pos());
        ignition::msgs::Set(z_marker.add_point(), point.Pos());

        ignition::msgs::Set(
            x_marker.add_point(),
            point.Pos() + point.Rot().RotateVector(ignition::math::Vector3d(1, 0, 0)));
        ignition::msgs::Set(
            y_marker.add_point(),
            point.Pos() + point.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0)));
        ignition::msgs::Set(
            z_marker.add_point(),
            point.Pos() + point.Rot().RotateVector(ignition::math::Vector3d(0, 0, 1)));
    }

    node.Request("/marker", x_marker);
    node.Request("/marker", y_marker);
    node.Request("/marker", z_marker);
}

void draw_enumeration(std::vector<ignition::math::Pose3d> const& trajectory) {
    ignition::transport::Node node;
    ignition::msgs::Marker text_marker;
    int id = 1;

    text_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    text_marker.set_type(ignition::msgs::Marker::TEXT);
    text_marker.set_ns("text");
    ignition::msgs::Set(text_marker.mutable_scale(),
                        ignition::math::Vector3d(0.2, 0.2, 0.2));

    for (auto& point : trajectory) {
        text_marker.set_id(id++);
        text_marker.set_text(std::to_string(id - 1));
        ignition::msgs::Set(text_marker.mutable_pose(), point);
        node.Request("/marker", text_marker);
    }
}

void draw_lines(std::vector<ignition::math::Pose3d> const& trajectory) {
    ignition::transport::Node node;
    ignition::msgs::Marker line_marker;

    line_marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    line_marker.set_type(ignition::msgs::Marker::LINE_LIST);
    line_marker.mutable_material()->mutable_script()->set_name("Gazebo/Orange");
    line_marker.set_ns("line");

    for (size_t i = 1; i < trajectory.size(); ++i) {
        ignition::msgs::Set(line_marker.add_point(), trajectory[i - 1].Pos());
        ignition::msgs::Set(line_marker.add_point(), trajectory[i].Pos());
    }

    node.Request("/marker", line_marker);
}

void draw_trajectory(std::vector<ignition::math::Pose3d> const& trajectory) {
    std::cout << "Visualizing trajectory of " << trajectory.size() << " points"
              << std::endl;

    draw_axis(trajectory);
    draw_enumeration(trajectory);
    draw_lines(trajectory);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected trajectory as an argument" << std::endl;
        return 1;
    }

    draw_trajectory(read_trajectory(argv[1]));
    return 0;
}
