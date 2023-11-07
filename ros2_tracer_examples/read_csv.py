import pandas as pd
import matplotlib.pyplot as plt

# Replace 'your_trace_data.csv' with the path to your CSV file
csv_file = 'trace_output.csv'

# Load the CSV data into a DataFrame
df = pd.read_csv(csv_file)

# Assume the CSV has columns 'timestamp' and 'value' that you want to plot
# Replace these with the actual column names from your CSV
plt.plot(df['timestamp'], df['value'])

# Add title and labels
plt.title('LTT Trace Graph')
plt.xlabel('Timestamp')
plt.ylabel('Value')

# Show the plot
plt.show()
