import argparse


def create_args(headless=False):
    gui = str(not headless).lower()
    headless = str(headless).lower()

    return f"""
    <arg name="use_sim_time" default="true" />
    <arg name="gui" default="{gui}" />
    <arg name="headless" default="{headless}" />
    <arg name="world_name" default="$(find simulation)/worlds/actually_empty_world.world" />
"""


WORLD_LAUNCH_FILE = """
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="debug" value="0" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="use_sim_time" value="$(arg use_sim_time)" />
    <arg name="headless" value="$(arg headless)" />
    <arg name="world_name" value="$(arg world_name)" />
    </include>
"""

GROUND_LAUNCH_FILE = """
    <include file="$(find simulation)/launch/spawn_cottage_blender_ground.launch" />
"""


def model_file(model=None):
    return f"""
    <include file="$(find simulation)/launch/spawn_cottage_blender_Cube.launch">
        {f"<arg name='model_filename' value='{model}' />" if model else ''}
    </include>
    """


def create_drones(num, basename="drone"):
    result = (
        f"""
        <group ns="drone{i}">
            <include file="$(find camera_controls)/launch/spawn_camera.launch">
            <arg name="model_name" value="drone{i}" />
            </include>
        </group>
        """
        for i in range(1, num + 1)
    )
    return "\n".join(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument(
        "--model", default=None, help="xacro model filename from simulation/urdf dir"
    )
    parser.add_argument(
        "--num", default=1, type=int, help="Number of drones in result configuration"
    )
    parser.add_argument("--headless", default=False, type=bool, help="Headless sim")

    args = parser.parse_args()

    result_text = f"""
<launch>
    {create_args(args.headless)}
    {WORLD_LAUNCH_FILE}
    {GROUND_LAUNCH_FILE}
    {model_file(args.model)}
    {create_drones(args.num)}
</launch>
"""

    print(result_text)
