import socket
import time
def send_tcp_packet(server_ip, server_port, file_path):
    try:
        # Create a TCP socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the server
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")

        # Open the file and process each line
        with open(file_path, 'r') as file:
            for line in file:
                # Strip any unnecessary whitespace or newlines from the line
                message = line.strip()
                
                # Send the message
                if message:  # Only send non-empty lines
                    client_socket.sendall(message.encode('utf-8'))
                    print(f"Sent: {message}")

                    # Optionally receive a response
                    try:
                        response = client_socket.recv(1024).decode('utf-8')
                        print(f"Received: {response}")
                    except socket.timeout:
                        print("No response received.")
                time.sleep(5)
        print("Finished sending all messages.")

    except socket.error as e:
        print(f"Socket error: {e}")

    except FileNotFoundError:
        print("Error: The specified file was not found.")

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        # Close the connection
        client_socket.close()
        print("Connection closed.")

# Read joint angles from the file and convert to the desired format
def convert_angles(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    with open(output_file, 'w') as f:
        for line in lines:
            # Split the line into individual angles
            angles = line.split()
            # Convert angles to integers or adjust the range as needed
            angles = [int(float(angle)) for angle in angles]
            # Write the angles in the desired format
            f.write(f"set_angles({', '.join(map(str, angles))}, 100)\n")

# Example usage
input_file = '/home/sanjay/Desktop/cobot600/joint_states_values.txt'  # Change to your input file path
output_file = 'angles.txt'  # Change to your desired output file path

print(f"Angles have been successfully converted and saved to {output_file}")
if __name__ == "__main__":
    convert_angles(input_file, output_file)
    print("Conversion completed!")
    # Replace these with your server details and file path
    SERVER_IP = "192.168.1.159"
    SERVER_PORT = 5001
    FILE_PATH = "angles.txt"  # Replace with the path to your text file

    send_tcp_packet(SERVER_IP, SERVER_PORT, FILE_PATH)
