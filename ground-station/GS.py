import tkinter as tk
import threading
import time

class GroundStationGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("NAKASHATRA Drone Ground Station")
        self.master.attributes("-fullscreen", True)
        self.master.config(bg="#1e1e1e")

        self.master.bind("<Escape>", self.exit_fullscreen)
        self.master.bind("<Up>", self.move_y_up)
        self.master.bind("<Down>", self.move_y_down)
        self.master.bind("<Left>", self.move_x_left)
        self.master.bind("<Right>", self.move_x_right)
        self.master.bind("w", self.increase_altitude)
        self.master.bind("s", self.decrease_altitude)

        self.mode = "Manual"
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.start_time = time.time()

        # Title
        self.title_label = tk.Label(master, text="NAKASHATRA DRONE GROUND STATION", fg="white", bg="#1e1e1e", font=("Arial", 32, "bold"))
        self.title_label.pack(pady=20)

        # Timer label
        self.timer_text = tk.StringVar()
        self.timer_label = tk.Label(master, textvariable=self.timer_text, fg="cyan", bg="#1e1e1e", font=("Courier", 18))
        self.timer_label.pack(pady=5)

        # Data display
        self.data_text = tk.StringVar()
        self.data_label = tk.Label(master, textvariable=self.data_text, fg="lime", bg="#1e1e1e", font=("Courier", 20), justify="center")
        self.data_label.pack(pady=20)

        # Button Frame
        self.button_frame = tk.Frame(master, bg="#1e1e1e")
        self.button_frame.pack(pady=20)

        self.arm_button = tk.Button(self.button_frame, text="ARM", width=15, height=2, font=("Arial", 14), command=self.arm)
        self.arm_button.grid(row=0, column=0, padx=10, pady=5)

        self.disarm_button = tk.Button(self.button_frame, text="DISARM", width=15, height=2, font=("Arial", 14), command=self.disarm)
        self.disarm_button.grid(row=0, column=1, padx=10, pady=5)

        self.takeoff_button = tk.Button(self.button_frame, text="TAKEOFF", width=15, height=2, font=("Arial", 14), command=self.takeoff)
        self.takeoff_button.grid(row=1, column=0, padx=10, pady=5)

        self.land_button = tk.Button(self.button_frame, text="LAND", width=15, height=2, font=("Arial", 14), command=self.land)
        self.land_button.grid(row=1, column=1, padx=10, pady=5)

        self.mode_button = tk.Button(master, text="SWITCH MODE", font=("Arial", 14), width=20, height=2, command=self.toggle_mode)
        self.mode_button.pack(pady=10)

        self.status_text = tk.StringVar()
        self.status_label = tk.Label(master, textvariable=self.status_text, fg="yellow", bg="#1e1e1e", font=("Arial", 16))
        self.status_label.pack(pady=10)

        threading.Thread(target=self.update_data_loop, daemon=True).start()
        threading.Thread(target=self.update_timer, daemon=True).start()

    # Button Functions
    def toggle_mode(self):
        self.mode = "Autonomous" if self.mode == "Manual" else "Manual"
        self.show_status(f"Mode switched to {self.mode}")

    def arm(self):
        self.show_status("Arming...")
        print("Drone armed (simulated)")
        self.master.after(1000, lambda: self.show_status("Drone Armed Successfully!"))

    def disarm(self):
        self.show_status("Disarming...")
        print("Drone disarmed (simulated)")
        self.master.after(1000, lambda: self.show_status("Drone Disarmed."))

    def takeoff(self):
        self.show_status("Takeoff Initiated...")
        print("Drone takeoff initiated (simulated)")
        self.master.after(1000, lambda: self.show_status("Drone in Air."))

    def land(self):
        self.show_status("Landing...")
        print("Drone landing initiated (simulated)")
        self.master.after(1000, lambda: self.show_status("Drone Landed."))

    def show_status(self, message):
        self.status_text.set(message)
        self.master.after(3000, lambda: self.status_text.set(""))

    def update_data_loop(self):
        while True:
            text = f"""
Telemetry Status: CONNECTED
Mode: {self.mode}

X Position: {self.x:.2f} m
Y Position: {self.y:.2f} m
Z (Altitude): {self.z:.2f} m
"""
            self.data_text.set(text)
            time.sleep(0.5)

    def update_timer(self):
        while True:
            elapsed = int(time.time() - self.start_time)
            mins = elapsed // 60
            secs = elapsed % 60
            self.timer_text.set(f"Time Elapsed: {mins:02d}:{secs:02d}")
            time.sleep(1)

    def exit_fullscreen(self, event=None):
        self.master.attributes("-fullscreen", False)

    # Movement Functions
    def move_x_right(self, event):
        self.x += 0.1
        # self.show_status("Moved Right (+X)")

    def move_x_left(self, event):
        self.x -= 0.1
        # self.show_status("Moved Left (-X)")

    def move_y_up(self, event):
        self.y += 0.1
        # self.show_status("Moved Forward (+Y)")

    def move_y_down(self, event):
        self.y -= 0.1
        # self.show_status("Moved Backward (-Y)")

    def increase_altitude(self, event):
        self.z += 0.1
        # self.show_status("Increased Altitude (+Z)")

    def decrease_altitude(self, event):
        self.z -= 0.1
        # self.show_status("Decreased Altitude (-Z)")

if __name__ == "__main__":
    root = tk.Tk()
    app = GroundStationGUI(root)
    root.mainloop()
