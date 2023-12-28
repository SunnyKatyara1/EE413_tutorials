#!/usr/bin/env python3
from parametric_policy import *
import tkinter as tk
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tkinter import filedialog, messagebox, Toplevel
from tkinter import simpledialog, PhotoImage
from PIL import Image as PilImage
from PIL import ImageTk
from tkinter import font as tkFont  # Import tkinter font module
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
import threading

# Global list to store trajectory points
trajectory_points = []

# Global flag to indicate if the robot should be drawing
is_drawing = True

def trajectory_callback(msg):
    global trajectory_points, is_drawing
    if is_drawing:
        # Add points to trajectory only if drawing is enabled
        for point in msg.points:
            trajectory_points.append((point.y, point.z))
            
def update_plot():
    # Update the plot with the trajectory points
    ydata = [point[0] for point in trajectory_points]
    zdata = [point[1] for point in trajectory_points]
    line.set_ydata(zdata)
    line.set_xdata(ydata)
    canvas.draw()

    root.after(100, update_plot)  # Schedule the next update


def move_to_home():
    group.set_named_target("homing")
    plan_result = group.plan()

    # If plan_result is a tuple and the first element is True, execute the plan
    if isinstance(plan_result, tuple) and plan_result[0]:
        group.execute(plan_result[1])  # Assuming the second element is the trajectory
    else:
        print("Failed to plan the movement to the home position.")
    
def reset_plot():
    global line, canvas, trajectory_points
    trajectory_points.clear()  # Clear the trajectory points
    line.set_xdata([])         # Clear the x-data of the line
    line.set_ydata([])         # Clear the y-data of the line
    canvas.draw()              # Redraw the canvas

def reset_robot_and_plot():
    move_to_home()
    reset_plot()

def play_video():
    # Open the video file
    cap = cv2.VideoCapture('/home/sunny/Pictures/demo.mp4')
    while True:  # Loop indefinitely
        ret, frame = cap.read()
        if ret:
            # Convert the image from BGR (OpenCV format) to RGB (Tkinter format)
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = PilImage.fromarray(cv2image)

            # Resize the image to fit the label
            imgtk = ImageTk.PhotoImage(image=img.resize((600, 340)))

            # Update the label with the new image
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)
            video_label.update()
        else:
            # If we reached the end of the video, reset to the beginning
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
    

def unity_ros_image_callback(ros_image):
    global unity_image_label, bridge

    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    pil_image = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    tk_image = ImageTk.PhotoImage(pil_image)

    unity_image_label.configure(image=tk_image)
    unity_image_label.image = tk_image



def ros_image_callback(ros_image):
    global image_label
    global bridge

    # Convert ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

    # Convert OpenCV image to PIL format
    pil_image = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

    # Convert PIL image to ImageTk format
    tk_image = ImageTk.PhotoImage(pil_image)

    # Update the image in Tkinter
    image_label.configure(image=tk_image)
    image_label.image = tk_image

def draw_name(name):
    current_pose = group.get_current_pose().pose
    current_pose.position.y -= 0.2  # Adjust based on your setup
    r_height = 0.1  # Change this to the desired height of the 'R'
    r_width = 0.05  # Change this to the desired width of the 'R'

    for i, letter in enumerate(name.lower()):
        print(current_pose)
        viapoint1 = []
        viapoint2 = []
        viapoint3 = []
        marker_id = i + 1  # Marker ID starts from 1 and increments for each letter
        marker = create_marker(marker_id)

        # Assuming each letter function is named like a_trajectory, b_trajectory, etc.
        trajectory_func = globals().get(f"{letter}_trajectory")
        if trajectory_func:
            waypoints = trajectory_func(r_height, r_width, [current_pose.position.x, current_pose.position.y, current_pose.position.z], marker)
            execute_trajectory(waypoints)
            marker_publisher.publish(marker)
            rospy.sleep(1)  # Wait between letters
            trajectory_points.append((None, None))

        else:
            print(f"No trajectory function for letter: {letter}")
        
        current_pose.position.x -= 0.02
        viapoint1.append(current_pose)
        execute_trajectory(viapoint1)
        current_pose.position.y += 0.1
        viapoint2.append(current_pose)
        execute_trajectory(viapoint2)
        current_pose.position.x += 0.02
        viapoint3.append(current_pose)
        execute_trajectory(viapoint3)


def on_enter_key(event):
    input_text = name_entry.get()
    name = extract_name(input_text)

    if name:
        draw_name(name)
    else:
        print("No valid name found in input.")
        
def open_file_dialog(model_name):
    # Open a file dialog to select a file for the chosen model
    file_path = filedialog.askopenfilename(title=f"Select file for {model_name}")
    if file_path:
        # You can handle the selected file here
        print(f"You selected {file_path} for {model_name}")

def imr_api_button_clicked():
    model_window = Toplevel(root)
    model_window.title("Select Model")

    # List of trained models
    models = ["PoseCNN", "PVNet", "FasterRCNN", "DOPE", "DenseFusion", "PointNet+", "DeepStitch"]

    # Create buttons for each model
    for model in models:
        button = tk.Button(model_window, text=model, 
                           command=lambda m=model: open_file_dialog(m))
        button.pack(pady=5)

def imr_programming_button_clicked():
    # Function to handle "IMr API" button click
    # Add your IMR API functionality here
    print("IMR API button clicked")

def imr_chatbot_button_clicked():
    # Function to handle "IMR Chatbot" button click
    imr_chatbot_window()

def imr_chatbot_window():
    # Create a new window for the IMR ChatBot
    chatbot_window = tk.Toplevel()
    chatbot_window.title("IMR ChatBot")

    # Create a text box in the new window
    chatbot_textbox = tk.Text(chatbot_window)
    chatbot_textbox.pack()

    # You can add more widgets or functionality to this window as needed
    
def browse_dataset(dataset):
    # Open a file dialog to select a file
    file_path = filedialog.askopenfilename(title=f"Select {dataset}")
    if file_path:
        # You can handle the selected file here
        messagebox.showinfo("File Selected", f"You selected {file_path}")

def on_button_click():
    # New window to select the dataset
    selection_window = Toplevel(root)
    selection_window.title("Select Dataset")

    # Option for the 6D Pose Dataset
    pose_dataset_btn = tk.Button(selection_window, text="6D Pose Dataset", 
                                 command=lambda: browse_dataset("6D Pose Dataset"))
    pose_dataset_btn.pack(pady=10)

    # Option for the Inspection Dataset
    inspection_dataset_btn = tk.Button(selection_window, text="Inspection Dataset", 
                                       command=lambda: browse_dataset("Inspection Dataset"))
    inspection_dataset_btn.pack(pady=10)

def create_gui():
    global root, canvas, line, image_label, unity_image_label, video_label
    root = tk.Tk()
    root.geometry("1200x600")
    root.title("IMR User Interface")
     
    # Convert the image to PhotoImage
    watermark_photo = PhotoImage(file="/home/sunny/Pictures/water_mark.png") 


    # Create a label to display the watermark
    # Adjust 'x' and 'y' to position the watermark as needed
    watermark_label = tk.Label(root, image=watermark_photo, bg="#28225F")  # Set bg to your window's background color
    watermark_label.place(x=0, y=0)  # Position the watermark
    root.configure(bg="#28225F")  # Set the background color
    # Load your logo (PNG or GIF)
    logo = PhotoImage(file="/home/sunny/Pictures/logos.png")  # Replace with the path to your logo

    # Add the logo to a label and display it
    logo_label = tk.Label(root, image=logo)
    logo_label.pack(pady=10)
    
    # Frame to hold text box and submit button
    entry_frame = tk.Frame(root, bg="#28225F")
    entry_frame.pack(padx=10, pady=10)
    
    
    global name_entry
    name_entry = tk.Text(entry_frame, height=5, width=50)  # Adjust height and width as needed
    name_entry.pack(side=tk.LEFT)


    # Function to handle submit button click
    def submit_name():
        input_text = name_entry.get("1.0", "end-1c")  # Get text from text box
        name = extract_name(input_text)

        if name:
            drawing_thread = threading.Thread(target=draw_name, args=(name,))
            drawing_thread.start()
        else:
            print("No valid name found in input.")

    
    # Create a submit button with an arrow symbol
    submit_button = tk.Button(entry_frame, text="->", command=submit_name)
    submit_button.pack(side=tk.LEFT, padx=5)
    reset_button = tk.Button(entry_frame, text="Reset", command=reset_robot_and_plot)
    reset_button.pack(side=tk.LEFT, padx=5)
    
    unity_image_label = tk.Label(root)  # Unity camera feed label
    unity_image_label.place(x=10, y=250)  # Adjust x and y for Unity feed

    image_label = tk.Label(root)  # Gazebo camera feed label
    image_label.place(x=630, y=250)  # Adjust x and y for Gazebo feed

    button_font = tkFont.Font(family="Helvetica", size=14, weight="bold")  # Adjust font settings as needed

    imr_api_button = tk.Button(root, text="IMR API", command=imr_api_button_clicked, height=2, width=40, font=button_font)
    imr_api_button.place(x=1350, y=350)  # Adjust x and y to place the button

    # "IMR Chatbot" button
    imr_chatbot_button = tk.Button(root, text="IMR Chatbot", command=imr_chatbot_button_clicked, height=2, width=40, font=button_font)
    imr_chatbot_button.place(x=1350, y=450)  # Adjust x and y to place the button
    
    imr_api_button = tk.Button(root, text="IMR Robot Programming", command=imr_programming_button_clicked, height=2, width=40, font=button_font)
    imr_api_button.place(x=1350, y=550)  # Adjust x and y to place the button
    
    photo = PhotoImage(file="/home/sunny/Pictures/imr_set.png")  # Replace with the path to your logo
    data_button = tk.Button(root, image=photo, command=on_button_click)
    data_button.place(x=1450, y=670)
    
    fig = Figure(figsize=(6.4, 3), dpi=100)
    ax = fig.add_subplot(111)
    fig.patch.set_facecolor('#28225F')  # Set the outer background color
    line, = ax.plot([], [], 'r-')  # Red line for the trajectory
    ax.set_xlim(-0.5, 0.8)  # Adjust as per your trajectory limits
    ax.set_ylim(0, 1)
    ax.tick_params(axis='both', colors='white')


    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.place(x=630, y=720)


    root.after(100, update_plot)
    
    video_label = tk.Label(root)
    video_label.place(x=10, y=700)
    thread2 = threading.Thread(target=play_video)
    thread2.start()
   
  

    root.mainloop()

# Main Execution
if __name__ == '__main__':


    # Create a bridge between ROS and OpenCV
    bridge = CvBridge()

    # Subscribe to the camera topic
    rospy.Subscriber("/rear_camera/rear_camera/image_raw", Image, ros_image_callback)
    rospy.Subscriber("/unity_camera/rgb/image_raw", Image, unity_ros_image_callback)
    rospy.Subscriber("/trajectory_marker", Marker, trajectory_callback)


    # Run the GUI in a separate thread
    gui_thread = threading.Thread(target=create_gui)
    gui_thread.start()
    
    rospy.spin()


