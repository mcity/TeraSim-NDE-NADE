import matplotlib.animation as animation
import matplotlib.pyplot as plt
import json
import SumoTrajVis
import os
from pathlib import Path
def visualizer(map_location, exp_repo, exp_name, export_path):
    final_state_json = {}
    maneuvers_json = {}
    name_with_core = "_".join(exp_name.split("_")[:-1])
    final_state_json_path = list(Path(f"{exp_repo}").glob(f"**/{name_with_core}_final_state.json"))[0]
    maneuver_challenges_json_path = list(Path(f"{exp_repo}").glob(f"**/{name_with_core}_maneuver_challenges.json"))[0]
    print(final_state_json_path)
    with open(final_state_json_path, 'r') as f:
        final_state_json = json.load(f)

    with open(maneuver_challenges_json_path, 'r') as f:
        maneuvers_json = json.load(f)

    # Load net file and trajectory file
    if not os.path.exists(export_path):
        os.system(f"mkdir -p {export_path}")
        os.system(f"chmod 755 {export_path}")
    
    if exp_name in final_state_json:
        net = SumoTrajVis.Net(f"{map_location}/mcity.net.xml")
        fcd_path = str(list(Path(f"{exp_repo}").glob(f"**/{exp_name}/fcd*.xml"))[0])
        trajectories = SumoTrajVis.Trajectories(fcd_path)
        # Set trajectory color for different vehicles
        for trajectory in trajectories:
            if trajectory.id == "CAV":
                trajectory.assign_colors_constant("#ff0000")
            else:
                trajectory.assign_colors_constant("#00FF00")

        # Show the generated trajectory video
        fig, ax = plt.subplots()
        artist_collection = net.plot(ax=ax)
        plot_time_interaval = trajectories.timestep_range()[-100:] # only plot 10s before the end of the trajectories, can be modified later
        intersections = SumoTrajVis.PotentialIntersection(
            duration=3,
            start_time=plot_time_interaval[0],
            end_time=plot_time_interaval[-1],
            final_state_json=final_state_json[exp_name],
            maneuvers_json=maneuvers_json[exp_name] if exp_name in maneuvers_json else None
        )

        a = animation.FuncAnimation(
            fig,
            trajectories.plot_points,
            frames=plot_time_interaval,
            interval=1,
            fargs=(ax, True, artist_collection.lanes, intersections),
            blit=False,
        )
        # plt.show()
        a.save(f"{export_path}/{exp_name}.mp4", writer=animation.FFMpegWriter(fps=10), dpi=300)
