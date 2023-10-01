import matplotlib.animation as animation
import matplotlib.pyplot as plt

import SumoTrajVis
import os

def visualizer(map_location, path_name, exp_name, export_path):
    # Load net file and trajectory file
    if not os.path.exists(export_path):
        os.system(f"mkdir -p {export_path}")
        os.system(f"chmod 755 {export_path}")
    net = SumoTrajVis.Net(f"{map_location}/maps/Mcity/mcity.net.xml")
    trajectories = SumoTrajVis.Trajectories(f"{path_name}/fcd_all.xml")
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
        final_state_json=f"{path_name}/final_state.json", 
        maneuvers_json=f"{path_name}/maneuver_challenges.json",
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


# if __name__ == "__main__":
#     # # why can't import shapely
#     # # the following code is runnable in the python script
#     import os
#     from tqdm import tqdm

#     map_location = "../"
#     experiment_record_repo = "/scratch/henryliu_root/henryliu98/shared_data/safetest/nde-obs-largescale"
#     export_path = "/scratch/henryliu_root/henryliu98/shared_data/safetest/nde-obs-largescale-video"
#     if not os.path.exists(export_path):
#         os.makedirs(export_path)

#     file_name = "check_list.txt"

#     with open(file_name) as f:
#         for line in tqdm(f.readlines()):
#             line = line.strip()
#             path_name = os.path.join(experiment_record_repo, line)
#             if os.path.isdir(path_name):
#                 visualizer(map_location, path_name, line, export_path)