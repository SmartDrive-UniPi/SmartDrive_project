import os
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk  # Make sure Pillow is installed for image handling
import subprocess

worlds_folder = "/home/ubuntu/psd_ws/src/psd_gazebo_sim/psd_gazebo_worlds/world"

teleop_src_folder = "/home/ubuntu/psd_ws/src/psd_gazebo_sim/custom_teleop/src"

def get_world_files():
    files = [f for f in os.listdir(worlds_folder) if f.endswith(".sdf")]
    return files

# Function to get the list of teleop options from the src folder
def get_teleop_options():
    files = [f.replace(".cpp", "") for f in os.listdir(teleop_src_folder) if f.endswith(".cpp")]
    return files

# Function to handle launching the processes in tmux
def launch_simulator_in_tmux(world_file_path, selected_teleop):
    try:
        # Colcon build
        print("Running colcon build in ~/psd_ws/")
        subprocess.run(["colcon", "build", "--symlink-install", "--parallel-workers", str(os.cpu_count())], check=True, cwd=os.path.expanduser("~/psd_ws/"))
        print("colcon build completed")

        # Create a new tmux session
        tmux_session_name = "psd_simulator"
        subprocess.run(["tmux", "new-session", "-d", "-s", tmux_session_name], check=True)

        # Pane 0: Gazebo Simulation
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 launch psd_vehicle_bringup gz_sim.launch.py world_file:=" + world_file_path, "C-m"], check=True)

        # Split the window vertically and run RViz2 in the new pane (Pane 1)
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{tmux_session_name}:0"], check=True)
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.1", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run rviz2 rviz2 -d /home/ubuntu/psd_ws/startup/rviz/psd_vehicle.rviz", "C-m"], check=True)

        # Split the window horizontally for the teleop node (Pane 2)
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{tmux_session_name}:0.1"], check=True)
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.2", f"source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run custom_teleop {selected_teleop}", "C-m"], check=True)

        # Pane for fake perception (Pane 3)
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{tmux_session_name}:0.2"], check=True)
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.3", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run psd_perception fake_perception", "C-m"], check=True)

        # Pane for fake SLAM (Pane 4)
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{tmux_session_name}:0.3"], check=True)
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.4", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run psd_slam fake_slam", "C-m"], check=True)

        # Attach to the tmux session so the user can view the nodes
        subprocess.run(["tmux", "attach-session", "-t", tmux_session_name])

    except subprocess.CalledProcessError as e:
        print(f"Error during setup: {e}")
        messagebox.showerror("Launch Error", f"Failed to launch simulation: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        messagebox.showerror("Unexpected Error", str(e))

# Function to handle the track and teleop selection
def select_track_and_teleop():
    selected_track = track_combo.get()
    selected_teleop = teleop_combo.get()

    if selected_track and selected_teleop:
        world_file_path = os.path.join(worlds_folder, selected_track)
        print(f"Selected track: {world_file_path}")
        print(f"Selected teleop: {selected_teleop}")
        # Launch the simulator and nodes in tmux
        launch_simulator_in_tmux(world_file_path, selected_teleop)
        root.destroy()
    else:
        messagebox.showwarning("Selection Error", "Please select both a track and a teleop method!")

# Create the main window
root = tk.Tk()
root.title("SmartDrive Project")

# Add the logo to the window (Make sure the path to the logo is correct)
logo_path = "/home/ubuntu/psd_ws/startup/material/logo.png"
if os.path.exists(logo_path):
    logo = Image.open(logo_path)
    logo = ImageTk.PhotoImage(logo)
    root.iconphoto(True, logo)

# Add an image inside the window
inside_image_path = "/home/ubuntu/psd_ws/startup/material/logo.png"
if os.path.exists(inside_image_path):
    inside_image = Image.open(inside_image_path)
    inside_image = ImageTk.PhotoImage(inside_image)
    image_label = tk.Label(root, image=inside_image)
    image_label.pack(pady=10)  # You can adjust padding as needed

# Create label and combo box for world files
label_track = ttk.Label(root, text="Select a Gazebo track:")
label_track.pack(pady=10)

track_combo = ttk.Combobox(root, values=get_world_files(), state="readonly")
track_combo.pack(pady=5)

# Create label and combo box for teleop options
label_teleop = ttk.Label(root, text="Select a custom teleop method:")
label_teleop.pack(pady=10)

teleop_combo = ttk.Combobox(root, values=get_teleop_options(), state="readonly")
teleop_combo.pack(pady=5)

# Create the launch button
launch_button = ttk.Button(root, text="Launch Simulator", command=select_track_and_teleop)
launch_button.pack(pady=20)

# Start the GUI event loop
root.mainloop()
