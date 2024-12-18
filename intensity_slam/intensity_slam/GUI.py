import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParkingLotGUI(Node):
    def __init__(self):
        super().__init__('parking_lot_gui')

        self.parking_status = {f"space_{i}": "EMPTY" for i in range(1, 17)}
        self.button_widgets = {}

        self.publisher_ = self.create_publisher(String, 'retrieve_car', 10)
        self.subscription = self.create_subscription(
            String, 'parking_status', self.update_parking_status, 10
        )

        self.root = tk.Tk()
        self.root.title("Parking Lot Manager")
        self.root.geometry("600x400")

        self.create_buttons()

        # Create a ROS2 timer to spin periodically
        self.create_timer(0.1, self.spin_once)

    def create_buttons(self):
        for i in range(1, 17):
            btn_text = f"Space {i}\nEMPTY"
            btn = tk.Button(
                self.root, text=btn_text, bg="green", width=15, height=3,
                command=lambda space=f"space_{i}": self.request_car_retrieval(space)
            )
            btn.grid(row=(i - 1) // 4, column=(i - 1) % 4, padx=5, pady=5)
            self.button_widgets[f"space_{i}"] = btn

    def update_parking_status(self, msg):
        data = msg.data.split(',')
        if len(data) != 2:
            self.get_logger().error(f"Invalid message format: {msg.data}")
            return

        space_id, number_plate = data
        if space_id in self.parking_status:
            self.parking_status[space_id] = number_plate if number_plate != "EMPTY" else "EMPTY"
            self.update_button(space_id, number_plate)

    def update_button(self, space_id, number_plate):
        btn = self.button_widgets.get(space_id)
        if btn:
            if number_plate == "EMPTY":
                btn.config(text=f"{space_id.replace('_', ' ').title()}\nEMPTY", bg="green")
            else:
                btn.config(text=f"{space_id.replace('_', ' ').title()}\n{number_plate}", bg="red")

    def request_car_retrieval(self, space_id):
        if self.parking_status[space_id] == "EMPTY":
            messagebox.showinfo("Info", f"Parking space {space_id.replace('_', ' ').title()} is empty.")
        else:
            msg = String()
            msg.data = space_id
            self.publisher_.publish(msg)
            messagebox.showinfo("Info", f"Requested retrieval for {space_id.replace('_', ' ').title()}.")

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    parking_lot_gui = ParkingLotGUI()
    try:
        rclpy.spin(parking_lot_gui)
        parking_lot_gui.run()
    finally:
        parking_lot_gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
