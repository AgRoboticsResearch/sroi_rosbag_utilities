import time
import subprocess
import logging
from gpiozero import Button
from e_paper_display import display_on_epaper
from constants import SYSTEM_STATES, ACTION_STATES
from main import RecordingSystem
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
scripts_path = "/home/brl/codes/raspberry_pi/upi_python/upi_info_display/scripts_oak/"
topic_info_path = "/home/brl/codes/rosbags/recording_status.txt"

class RecordingSystemOAK(RecordingSystem):
    def __init__(self, scripts_path):
        super().__init__(scripts_path)
    
if __name__ == "__main__":
    record_system = RecordingSystemOAK(scripts_path)
    record_system.run()