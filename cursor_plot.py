import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cv2
import threading
import numpy as np

# Configure the csv file
CSV_PATH = 'Test1.csv'  # Path
X_COLUMN = 'Time'
Y_COLUMNS = ['Heading', 'Oscillation', 'Current']
ANIMATION_DURATION = 43
FPS = 20


df = pd.read_csv(CSV_PATH)
x_data = df[X_COLUMN].values.astype('float64')
y_data = [df[col].values for col in Y_COLUMNS]

#Plot settings
fig, ax = plt.subplots()
lines = [ax.plot(x_data, y, label=col)[0] for y, col in zip(y_data, Y_COLUMNS)]
cursor = ax.axvline(x=x_data[0], color='red', linestyle='--', label='Cursor')
ax.legend()
ax.set_xlabel(X_COLUMN)
ax.set_ylabel('Values')
ax.set_title('Graph')

# Animation
frame_count = ANIMATION_DURATION * FPS
x_min, x_max = x_data.min(), x_data.max()
x_range = x_max - x_min

def update(frame):
    t = frame / frame_count
    cursor_x = x_min + t * x_range
    cursor.set_xdata(cursor_x)
    return [cursor]

#def play_video():
#    cap = cv2.VideoCapture("SmallFins1.mp4") # name of the video
#    if not cap.isOpened():
#        print("Error: Could not open video.")
#        return
#    while cap.isOpened():
#        ret, frame = cap.read()
#        if not ret:
#            break
#        cv2.imshow("Video Playback", frame)
#        if cv2.waitKey(30) & 0xFF == ord('q'):  # Press 'q' to quit
#            break
#    cap.release()
#    cv2.destroyAllWindows()

ani = FuncAnimation(fig, update, frames=frame_count, interval=1000/FPS, blit=True)
plt.show()

#play_video()
#plot_thread.join()