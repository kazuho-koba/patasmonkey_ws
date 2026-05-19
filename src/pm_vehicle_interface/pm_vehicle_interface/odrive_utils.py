import odrive
from odrive.utils import dump_errors
import time


class ODriveUtils:
    @staticmethod
    def find_odrive():
        """Search for an ODrive device and return the instance."""
        print("Searching for ODrive...")
        odrv = odrive.find_any(timeout=1)
        if odrv is None:
            raise Exception("ODrive not found!")
        print(f"ODrive connected! Voltage: {odrv.vbus_voltage:.2f}V", flush=True)
        return odrv

    @staticmethod
    def clear_odrive_errors(odrv):
        """Clear errors on the ODrive device."""
        try:
            print("Clearing ODrive errors...", flush=True)
            odrv.clear_errors()
            print("Errors cleared.", flush=True)
        except Exception as e:
            print(f"Error clearing ODrive: {e}")

    @staticmethod
    def check_odrive_errors(odrv):
        """Check and print ODrive error status."""
        try:
            print("Checking ODrive errors...", flush=True)
            dump_errors(odrv, True)
        except Exception as e:
            print(f"Error checking ODrive: {e}")

    @staticmethod
    def reboot_odrive(odrv):
        """Reboot the ODrive device."""
        try:
            print("Rebooting ODrive...")
            odrv.reboot()
            time.sleep(3)  # Wait for ODrive to restart
            print("ODrive rebooted.")
        except Exception as e:
            print(f"Error rebooting ODrive: {e}")
