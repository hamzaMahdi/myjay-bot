import socket
import time
import random

# UBIDOTS PARAMETERS
SERVER = "things.ubidots.com"
PORT = 9012
BUFFER_SIZE = 1024
TIMEOUT = 10
TOKEN = #config removed for privacy
DEVICE_NAME = #config removed for privacy
DEVICE_LABEL = #config removed for privacy
VARIABLE_LABEL = "joy"
DELAY = 1  # Delay in seconds

def send_tcp_packet(payload, token=TOKEN):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.connect_ex((SERVER, PORT))
        s.settimeout(TIMEOUT)
        message = "test/1|POST|{}|".format(token)
        message = "{}{}|end".format(message, payload)
        s.send(message.encode())
        data = s.recv(BUFFER_SIZE)
        print("[INFO] Server Answer: {}".format(data.decode()))
        if data.decode() != 'OK':
            print("[ERROR] There was an error sending your dot")
        else:
            print("[INFO] Sucessfull posted!")
        s.close()
    except Exception as e:
        print("[ERROR] There was an error sending your dot")
        print("details: {}".format(e))

def main():

    # Simulates sensor values
    sensor_value_1 = random.random() * 100

    # Builds Payload and topic
    payload = "{}:{}=>{}:{}".format(DEVICE_LABEL, DEVICE_NAME,
                                    VARIABLE_LABEL, sensor_value_1
                                    )

    print("[INFO] Sending data")
    send_tcp_packet(payload)

if __name__ == "__main__":
    while True:
        main()
        time.sleep(DELAY)