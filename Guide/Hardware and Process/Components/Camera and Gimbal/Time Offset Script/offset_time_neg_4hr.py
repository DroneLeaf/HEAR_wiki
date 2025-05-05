import os
import time
from datetime import datetime, timedelta

def adjust_creation_time(directory=".", hours=4):
    offset = timedelta(hours=hours)
    
    for filename in os.listdir(directory):
        if filename.lower().endswith((".jpg", ".jpeg", ".png", ".gif", ".bmp", ".tiff")):
            filepath = os.path.join(directory, filename)
            
            # Get current modification and access times
            stat_info = os.stat(filepath)
            modified_time = stat_info.st_mtime
            access_time = stat_info.st_atime
            
            new_time = modified_time - offset.total_seconds()
            
            # Apply the new timestamps
            os.utime(filepath, (access_time, new_time))
            
            print(f"Updated: {filename} -> New Modified Time: {datetime.fromtimestamp(new_time)}")

if __name__ == "__main__":
    adjust_creation_time()