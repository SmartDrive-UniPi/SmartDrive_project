import os
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from PIL import Image, ImageTk
import subprocess
import time
import xml.etree.ElementTree as ET

# Folder where world files are located
worlds_folder = "/home/ubuntu/psd_ws/src/psd_gazebo_sim/psd_gazebo_worlds/world"
teleop_src_folder = "/home/ubuntu/psd_ws/src/psd_gazebo_sim/custom_teleop/src"
screenshots_folder = "/home/ubuntu/psd_ws/screenshots/"
gz_path = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"

# Ensure screenshots folder exists
if not os.path.exists(screenshots_folder):
    os.makedirs(screenshots_folder)

# Function to get the list of world files
def get_world_files():
    return [f for f in os.listdir(worlds_folder) if f.endswith(".sdf")]

# Function to get teleop options
def get_teleop_options():
    return [f.replace(".cpp", "") for f in os.listdir(teleop_src_folder) if f.endswith(".cpp")]

# Inject camera sensor into SDF
def inject_camera_sensor(sdf_file):
    try:
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        # Check if a camera is already present
        if root.find(".//sensor[@type='camera']") is not None:
            return True  # Camera already exists

        # Inject camera sensor into the first model
        world = root.find('world')
        if world is None:
            return False

        camera_model = ET.Element("model", name="camera_model")
        pose = ET.Element("pose")
        pose.text = "0 0 10 0 0 0"  # Set position of the camera
        camera_model.append(pose)

        link = ET.Element("link", name="link")
        camera_sensor = ET.Element("sensor", name="camera_sensor", type="camera")
        camera = ET.Element("camera")
        horizontal_fov = ET.Element("horizontal_fov")
        horizontal_fov.text = "1.3962634"
        camera.append(horizontal_fov)

        image = ET.Element("image")
        width = ET.Element("width")
        width.text = "800"
        height = ET.Element("height")
        height.text = "600"
        format = ET.Element("format")
        format.text = "R8G8B8"
        image.extend([width, height, format])
        camera.append(image)

        clip = ET.Element("clip")
        near = ET.Element("near")
        near.text = "0.1"
        far = ET.Element("far")
        far.text = "100"
        clip.extend([near, far])
        camera.append(clip)

        save = ET.Element("save", enabled="true")
        save_path = ET.Element("save_path")
        save_path.text = screenshots_folder
        camera.append(save)
        camera_sensor.append(camera)
        link.append(camera_sensor)
        camera_model.append(link)
        world.append(camera_model)

        tree.write(sdf_file)
        return True

    except Exception as e:
        print(f"Failed to inject camera sensor into {sdf_file}: {e}")
        return False

# Generate and show the image preview
def show_track_image(selected_track):
    try:
        world_file_path = os.path.join(worlds_folder, selected_track)
        screenshot_file = os.path.join(screenshots_folder, f"{selected_track}.png")

        # Inject camera sensor
        if inject_camera_sensor(world_file_path):
            print(f"Camera sensor injected into {selected_track}")

        # Run Ignition Gazebo and take the screenshot
        if not os.path.exists(screenshot_file):
            print(f"Generating screenshot for {selected_track}...")
            # Run Ignition Gazebo headless and let the camera sensor save the image
            subprocess.run([gz_path, "sim", world_file_path, "-r"], check=True)
            time.sleep(5)  # Give enough time for the world to load and save the image
            subprocess.run(["pkill", "gz"], check=True)  # Kill the simulation after saving the screenshot

        # Display the screenshot in the GUI
        if os.path.exists(screenshot_file):
            img = Image.open(screenshot_file)
            img = img.resize((300, 200), Image.ANTIALIAS)  # Resize for display
            img = ImageTk.PhotoImage(img)
            image_label.config(image=img)
            image_label.image = img
        else:
            messagebox.showerror("Error", f"Screenshot not generated for {selected_track}")

    except Exception as e:
        print(f"Failed to generate preview for {selected_track}: {e}")
        messagebox.showerror("Error", f"Failed to generate preview for {selected_track}: {e}")

# Handle track and teleop selection
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

        # First split the window vertically into two panes (50% up and 50% down)
        subprocess.run(["tmux", "split-window", "-v", "-t", f"{tmux_session_name}:0"], check=True)

        # Create three panes on the top half
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{tmux_session_name}:0.0"], check=True)  # Pane 1 (Middle)
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{tmux_session_name}:0.1"], check=True)  # Pane 2 (Right)
        subprocess.run(["tmux", "select-layout", "-t", f"{tmux_session_name}:0", "tiled"], check=True)  # Adjust to tiled layout (3 panes on top)

        # Create two panes on the bottom half (Pane 3 is bottom left and Pane 4 is bottom right)
        subprocess.run(["tmux", "split-window", "-h", "-t", f"{tmux_session_name}:0.1"], check=True)
        subprocess.run(["tmux", "select-layout", "-t", f"{tmux_session_name}:0.1", "tiled"], check=True)  # Adjust to tiled layout (2 panes on bottom)

        # Now assign the ROS2 commands to each pane
        # Top-left pane (Pane 0): Gazebo Simulation
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.0", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 launch psd_vehicle_bringup gz_sim.launch.py world_file:=" + world_file_path, "C-m"], check=True)

        # Top-middle pane (Pane 1): RViz2
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.1", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run rviz2 rviz2 -d /home/ubuntu/psd_ws/startup/rviz/psd_vehicle.rviz", "C-m"], check=True)

        # Top-right pane (Pane 2): Custom Teleop
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.2", f"source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run custom_teleop {selected_teleop}", "C-m"], check=True)

        # Bottom-left pane (Pane 3): Fake Perception
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.3", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run psd_perception fake_perception", "C-m"], check=True)

        # Bottom-right pane (Pane 4): Fake SLAM
        subprocess.run(["tmux", "send-keys", "-t", f"{tmux_session_name}:0.4", "source ~/.bashrc; source /home/ubuntu/psd_ws/install/setup.bash; ros2 run psd_slam fake_slam", "C-m"], check=True)

        # Attach to the tmux session so the user can view the nodes
        subprocess.run(["tmux", "attach-session", "-t", tmux_session_name])

    except subprocess.CalledProcessError as e:
        print(f"Error during setup: {e}")
        messagebox.showerror("Launch Error", f"Failed to launch simulation: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
        messagebox.showerror("Unexpected Error", str(e))

# Create the main window
root = tk.Tk()
root.title("Select Gazebo Track and Custom Teleop")

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

# Create image label to display the preview
image_label = tk.Label(root)
image_label.pack(pady=10)

# Create the launch button
launch_button = ttk.Button(root, text="Launch Simulator", command=select_track_and_teleop)
launch_button.pack(pady=20)

# Bind track selection to preview function
track_combo.bind("<<ComboboxSelected>>", lambda event: show_track_image(track_combo.get()))

# Start the GUI event loop
root.mainloop()
