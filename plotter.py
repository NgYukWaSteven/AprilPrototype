import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file without headers
df = pd.read_csv('received_data.csv', header=None)

# Assign column names manually
df.columns = ['timestamp', 'pitchA', 'rollA', 'pitchB', 'rollB']

# Extract columns
time = df['timestamp']
pitchA = df['pitchA']
rollA = df['rollA']
pitchB = df['pitchB']
rollB = df['rollB']

# Plotting
plt.figure(figsize=(12, 6))

# Pitch subplot
plt.subplot(2, 1, 1)
plt.plot(time, pitchA, label='Pitch A')
plt.plot(time, pitchB, label='Pitch B')
plt.title('Pitch Over Time')
plt.ylabel('Pitch (deg)')
plt.legend()
plt.grid(True)

# Roll subplot
plt.subplot(2, 1, 2)
plt.plot(time, rollA, label='Roll A')
plt.plot(time, rollB, label='Roll B')
plt.title('Roll Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Roll (deg)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
