import matplotlib.animation as animation
import matplotlib.pyplot as plt

import SumoTrajVis

import argparse

parser = argparse.ArgumentParser(description='Run simulation.')
parser.add_argument('--path', type=str, help='path to directory', default="path/to/output")
parser.add_argument('--dir', type=str, help='output directory', default="output")
parser.add_argument('--mode', type=str, help='the negligence mode.', default="test")
parser.add_argument('--file', type=str, help='simulation directory', default="0_0")
args = parser.parse_args()


# Load net file and trajectory file
net = SumoTrajVis.Net("./maps/Mcity/mcity.net.xml")

# use the following line if you want to use the default output directory
if args.path == "path/to/output":
    path_name=f"{args.dir}/{args.mode}/raw_data/{args.mode}_{args.file}"
else:
    path_name = args.path

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
plot_time_interaval = trajectories.timestep_range()[-300:] # only plot 30s before the end of the trajectories, can be modified later
intersections = SumoTrajVis.PotentialIntersection(
    duration=2,
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
a.save(f"{path_name}/{args.mode}_{args.file}.mp4", writer=animation.FFMpegWriter(fps=10), dpi=300)