from communicator import TcpCommunicator

if __name__ == "__main__":
    com = TcpCommunicator(ip="192.168.1.228", port=50001)
    com.connect()
    
    while True:
        # rx = com.recv()
        # print(f"Received data: {rx}")
        tx = input("Input data: ")
        com.send(tx)
        
        if tx == "q":
            com.close()
            break
