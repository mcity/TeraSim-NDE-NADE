import matplotlib.animation as animation
import matplotlib.pyplot as plt
from pathlib import Path
import SumoTrajVis
from tqdm import tqdm
import os
# Load net file and trajectory file
net = SumoTrajVis.Net("./maps/Mcity_safetest/mcity.net.xml")
traj_folder = Path("./output")
fcd_files = list(traj_folder.rglob("*fcd_all.xml"))
os.makedirs("./videos", exist_ok=True)

for fcd_file in tqdm(fcd_files):
    try:
        fcd_name = fcd_file.parent.name
        trajectories = SumoTrajVis.Trajectories(str(fcd_file))
        # Set trajectory color for different vehicles
        for trajectory in trajectories:
            if trajectory.id == "CAV":
                trajectory.assign_colors_constant("#ff0000")
            else:
                trajectory.assign_colors_constant("#00FF00")

        # Show the generated trajectory video
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')
        artist_collection = net.plot(ax=ax)
        plot_time_interaval = trajectories.timestep_range()[-100:] # only plot 10s before the end of the trajectories, can be modified later
        a = animation.FuncAnimation(
            fig,
            trajectories.plot_points,
            frames=plot_time_interaval,
            interval=1,
            fargs=(ax, True, artist_collection.lanes),
            blit=False,
        )
        # plt.show()
        a.save(f"./videos/{fcd_name}.mp4", writer=animation.FFMpegWriter(fps=10), dpi=300)
    except:
        pass