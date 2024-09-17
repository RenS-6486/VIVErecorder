import os
from datetime import datetime


def get_creation_time(file_path):
    # Get the creation time in seconds since epoch
    creation_time = os.path.getctime(file_path)

    # Convert it to a human-readable format
    creation_time_formatted = datetime.fromtimestamp(creation_time).strftime('%Y-%m-%d %H:%M:%S')
    print(creation_time)
    return creation_time_formatted


# Replace 'your_video_file.mp4' with the path to your video file
file_path = 'test2.mp4'
print(f"Creation Time: {get_creation_time(file_path)}")
