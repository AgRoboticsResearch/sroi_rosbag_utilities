import time
import subprocess
import logging
from gpiozero import Button
from e_paper_display import display_on_epaper
from constants import SYSTEM_STATES, ACTION_STATES

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
scripts_path = "/home/brl/codes/raspberry_pi/upi_python/upi_info_display/scripts/"
topic_info_path = "/home/brl/codes/rosbags/recording_status.txt"

class RecordingSystem:
    def __init__(self, scripts_path):
        self.scripts_path = scripts_path
        print("Recording System initialized")
        print("Script path: ", self.scripts_path)
        self.cur_system_state = SYSTEM_STATES[0]
        self.cur_action_state = ACTION_STATES[0]
        self.button = Button(15, pull_up=True)
        self.pressed_time = None
        self.long_press_duration = 3
        self.system_started = False
        self.clean_topic_info()
        self.stop_system()
        self.start_system()
        self.start_action_server()
    
    def clean_topic_info(self):
        with open(topic_info_path, "w") as f:
            f.write("No topic info")

    def run_script_with_display(self, scripts_path, start_message, end_message, sleep_time, iversion=False):
        logging.info(start_message)
        display_on_epaper(start_message, update_time=True, battery_info=True, disk_info=True, topic_info=True, iversion=iversion)
        try:
            self.run_script(scripts_path)
            time.sleep(sleep_time)
            logging.info(end_message)
            display_on_epaper(end_message, update_time=True, battery_info=True, disk_info=True, topic_info=True, iversion=iversion)
        except Exception as e:
            logging.error(f"Error during {start_message.lower()}: {e}")
            display_on_epaper(f"Error: {e}", update_time=True, battery_info=True, disk_info=True, topic_info=True, iversion=iversion)

    def start_system(self):
        self.run_script_with_display(self.scripts_path + "/start_system.sh", "System \n Starting ...", "System \n Started", 10)
        self.system_started = True

    def stop_system(self):
        self.run_script_with_display(self.scripts_path + "/stop_system.sh", "System \n Stopping ...", "System \n Stopped", 3)

    def start_action_server(self):
        self.run_script_with_display(self.scripts_path + "/start_action_server.sh", "Action Server \n Starting ...", "Action Server \n Started", 1)

    def start_recording(self):
        self.run_script_with_display(self.scripts_path + "/record_rosbag.sh", "Recording \n Starting ...", "Recording \n Started", 3)

    def stop_recording(self):
        self.run_script_with_display(self.scripts_path + "/stop_record_rosbag.sh", "Recording \n Stopping ...", "Recording \n Stopped", 3)

    def run_script(self, scripts_path):
        try:
            subprocess.run([scripts_path], check=True)
            logging.info("Script executed successfully")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to execute script {scripts_path}: {e}")
            raise
        except Exception as e:
            logging.error(f"Unexpected error while executing script {scripts_path}: {e}")
            raise

    def handle_long_press(self):
        logging.info(f"Current system state: {self.cur_system_state}")
        if self.cur_system_state == SYSTEM_STATES[0]:
            logging.info("Long press detected. Starting recording ...")
            self.start_recording()
            self.cur_system_state = SYSTEM_STATES[1]
        else:
            logging.info("Long press detected. Stopping recording ...")
            self.stop_recording()
            self.cur_system_state = SYSTEM_STATES[0]

    def handle_short_press(self):
        if self.system_started:
            action_state_bool = not self.cur_action_state
            logging.info(f"Action: {action_state_bool}")
            try:
                subprocess.Popen([
                    "docker", "exec", "-i", "upi_system", "bash", "-c",
                    f"source /ros_entrypoint.sh && rosservice call /upi/status/toggle_srv {action_state_bool}"
                ])
                self.cur_action_state = action_state_bool
                display_on_epaper(
                    self.cur_system_state + f" \n Action: {self.cur_action_state}",
                    update_time=True, iversion=action_state_bool, battery_info=True, disk_info=True, topic_info=True
                )
            except Exception as e:
                logging.error(f"Failed to toggle action state: {e}")
                display_on_epaper("Error: Action toggle failed", update_time=True, battery_info=True, disk_info=True, topic_info=True)
        else:
            logging.warning("Docker container is not running or container_id is not available.")
            display_on_epaper("Error: Docker not running", update_time=True, battery_info=True, disk_info=True, topic_info=True)

    def run(self):
        try:
            while True:
                if self.button.is_pressed:
                    if self.pressed_time is None:
                        self.pressed_time = time.time()
                else:
                    if self.pressed_time is not None:
                        press_duration = time.time() - self.pressed_time
                        if press_duration < self.long_press_duration:
                            self.handle_short_press()
                        else:
                            self.handle_long_press()
                        self.pressed_time = None
                time.sleep(0.1)
        except KeyboardInterrupt:
            logging.info("Shutting down system...")
            self.stop_system()

if __name__ == "__main__":
    record_system = RecordingSystem(scripts_path)
    record_system.run()
