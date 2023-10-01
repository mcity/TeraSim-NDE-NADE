import matplotlib.animation as animation
import matplotlib.pyplot as plt
import json
import SumoTrajVis
import os

def visualizer(map_location, exp_repo, exp_name, export_path):
    final_state_json = {}
    maneuvers_json = {}
    name_with_core = "_".join(exp_name.split("_")[:-1])
    with open(f"{exp_repo}/final_state/{name_with_core}_final_state.json", 'r') as f:
        final_state_json = json.load(f)

    with open(f"{exp_repo}/maneuver_challenges/{name_with_core}_maneuver_challenges.json", 'r') as f:
        maneuvers_json = json.load(f)

    # Load net file and trajectory file
    if not os.path.exists(export_path):
        os.system(f"mkdir -p {export_path}")
        os.system(f"chmod 755 {export_path}")
    
    if exp_name in final_state_json:
        net = SumoTrajVis.Net(f"{map_location}/maps/Mcity/mcity.net.xml")
        trajectories = SumoTrajVis.Trajectories(f"{exp_repo}/{exp_name}/fcd_all.xml")
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
