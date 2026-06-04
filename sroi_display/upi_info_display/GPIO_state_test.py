from gpiozero import Button
import time

# Define the pin number (BCM numbering)
button = Button(15, pull_up=True)  # gpiozero uses pull_up=True for pull-ups, so use pull_up=False for pull-downs

# Start a timer
start_time = time.time()

# Run a loop for 10 seconds
while True:
    # Check if the button is pressed or not
    if button.is_pressed:
        print("Button is pressed")
    else:
        print("Button is not pressed")

    # Check if 10 seconds have passed
    if time.time() - start_time > 10:
        print("Exiting after 10 seconds...")
        break  # Exit the loop

    time.sleep(0.1)  # Small delay to prevent busy-waiting
