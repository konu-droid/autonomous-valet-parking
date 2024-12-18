import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from avp_resources.msg import ButtonStates

class ButtonGUI(Node):
    def __init__(self):
        super().__init__('button_gui')

        # ROS2 Publishers and Subscribers
        self.button_state_pub = self.create_publisher(String, 'button_status', 10)
        self.button_update_sub = self.create_subscription(ButtonStates, 'update_buttons', self.update_button_states, 10)

        # Initialize the array that determines button state
        self.button_states = ["vacant"] * 16  # "vacant" or "occupied"

        # Create the main window
        self.root = tk.Tk()
        self.root.title("Dynamic Button GUI")

        # Create a grid of buttons
        self.buttons = []
        for i in range(16):
            button = tk.Button(
                self.root,
                text="Vacant",
                bg="green",
                fg="white",
                command=lambda i=i: self.on_button_click(i),
                width=10,
                height=2,
            )
            button.grid(row=i // 4, column=i % 4, padx=5, pady=5)
            self.buttons.append(button)

        # Add the Park button
        park_button = tk.Button(
            self.root,
            text="Park",
            bg="blue",
            fg="white",
            command=self.on_park_button_click,
            width=10,
            height=2,
        )
        park_button.grid(row=4, column=0, columnspan=4, pady=10)

        # Initialize button states
        self.update_buttons()

    def update_buttons(self):
        """Update the buttons based on the button_states array."""
        for i, button in enumerate(self.buttons):
            if self.button_states[i] == "vacant":
                button.config(bg="green", text="Vacant")
            else:
                button.config(bg="red", text="Occupied")

    def on_button_click(self, button_id):
        """Handle button click event."""
        if self.button_states[button_id] == "occupied":
            msg = String()
            msg.data = f"Button ID: {button_id}, Text: Occupied"
            self.button_state_pub.publish(msg)
            self.button_states[button_id] = "vacant"
        self.update_buttons()

    def on_park_button_click(self):
        """Handle the Park button click event."""
        vacant_button_id = next((i for i, state in enumerate(self.button_states) if state == "vacant"), None)
        if vacant_button_id is not None:
            msg = String()
            msg.data = f"Button ID: {vacant_button_id}, Text: Park"
            self.button_state_pub.publish(msg)
        print("Success")

    def update_button_states(self, msg):
        """Update the button states based on the received ROS2 topic."""
        if len(msg.data) == 16:
            self.button_states = msg.data
            self.update_buttons()

    def start_gui(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    button_gui = ButtonGUI()

    # Run the GUI in a separate thread
    import threading
    gui_thread = threading.Thread(target=button_gui.start_gui)
    gui_thread.start()

    # Spin the ROS2 node
    try:
        rclpy.spin(button_gui)
    except KeyboardInterrupt:
        pass
    finally:
        button_gui.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
