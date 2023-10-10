import requests

# Define the URL of your Flask server
server_url = " http://127.0.0.1:5000/"

# Coordinates to send
coordinates = "12"  # This should be a string like "12" for row 1 and column 2

# Create a dictionary with the coordinates
data = {'pos': coordinates}

# Make a POST request to the server
response = requests.post(server_url, json=data)
print(response)
# Check the response from the server
if response.status_code == 200:
    print("Coordinates sent successfully.")
    print(response.text)  # This will print the response from your Flask server
else:
    print("Failed to send coordinates.")
