import socket
import json
import readline


SOCK = "/tmp/pe_sock"

with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
    sock.connect(SOCK)
    while True:
        print("act > ", end="")
        act = input()
        if act == "quit":
            break

        print("data > ", end="")
        raw_data = input()

        try:
            data = json.loads(raw_data)
        except Exception as e:
            if raw_data:
                data = raw_data
            else:
                data = "empty"

        json_data = json.dumps({"act": act, "data": data})
        sock.sendall(bytes(json_data + "\n", "utf-8"))
        print("Sent:     {}".format(json_data))

        received = str(sock.recv(1024), "utf-8")
        print(f"Received: {received}")

        if received == "exit":
            break
