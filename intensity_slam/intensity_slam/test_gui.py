import tkinter as tk

# Initialize the array that determines button state
button_states = ["vacant"] * 16  # "vacant" or "occupied"

def update_buttons():
    """Update the buttons based on the button_states array."""
    for i, button in enumerate(buttons):
        if button_states[i] == "vacant":
            button.config(bg="green", text="Vacant")
        else:
            button.config(bg="red", text="SN66 XMZ")

def on_button_click(button_id):
    """Handle button click event."""
    global button_states

    if button_states[button_id] == "occupied":
        print(f"Button ID: {button_id}, Text: Occupied")
        button_states[button_id] = "vacant"
    update_buttons()

def on_park_button_click():
    """Handle the Park button click event."""
    print("Success")

# Create the main window
root = tk.Tk()
root.title("Autonomous Valet Parking GUI")

# Create a grid of buttons
buttons = []
for i in range(16):
    button = tk.Button(
        root,
        text="Vacant",
        bg="green",
        fg="white",
        command=lambda i=i: on_button_click(i),
        width=10,
        height=2,
    )
    button.grid(row=i // 4, column=i % 4, padx=5, pady=5)
    buttons.append(button)

# Add the Park button
park_button = tk.Button(
    root,
    text="Park",
    bg="blue",
    fg="white",
    command=on_park_button_click,
    width=10,
    height=2,
)
park_button.grid(row=4, column=0, columnspan=4, pady=10)

# Function to modify the state of a specific button (example usage)
def occupy_button(button_id):
    if 0 <= button_id < len(button_states):
        button_states[button_id] = "occupied"
        update_buttons()

# Initialize button states
update_buttons()

# Example: Setting button 3 to occupied
occupy_button(3)

# Start the GUI loop
root.mainloop()
