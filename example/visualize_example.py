import matplotlib.animation as animation
import matplotlib.pyplot as plt
from pathlib import Path
import SumoTrajVis
from tqdm import tqdm
import os
# Load net file and trajectory file
net = SumoTrajVis.Net("./example/maps/Mcity_safetest/mcity.net.xml")
traj_folder = Path("/gpfs/accounts/henryliu_root/henryliu98/shared_data/safetest-nade/ITE_refactor_test_v2/")
raw_data_folder = traj_folder / "raw_data"
raw_data_file_folder_list = list(raw_data_folder.glob("*"))
# filter the raw_data_file_folder_list, only keep the folders that have fcd_all.xml file
raw_data_file_folder_list = [folder for folder in raw_data_file_folder_list if list(folder.glob("*fcd_all.xml"))]

video_path = traj_folder / "videos"
if not os.path.exists(video_path):
    os.makedirs(video_path)

for raw_data_file_folder in tqdm(raw_data_file_folder_list):
    # try:
    if 1:
        fcd_file = list(raw_data_file_folder.glob("*fcd_all.xml"))[0]
        collision_file_path = list(raw_data_file_folder.glob("*collision.xml"))[0]
        # if collision file path exists and "victim" is in the content of the collision file, then we will use this fcd file to generate video
        with open(collision_file_path, "r") as f:
            collision_file_content = f.read()
        if "victim" not in collision_file_content:
            continue
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
        video_name = raw_data_file_folder.name + ".mp4"
        save_video_path = video_path / video_name
        a.save(save_video_path, writer=animation.FFMpegWriter(fps=10), dpi=300)
    # except Exception as e:
    #     print(e)