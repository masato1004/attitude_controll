import socket
import time


class TcpCommunicator():
    def __init__(self, ip="localhost", port=50001, hosting=False):
        self.ip = ip
        self.port = port
        self.host = hosting
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def connect(self):
        if self.host:
            self.socket.bind((self.ip, self.port))
            self.socket.listen(1)
            self.soc, self.addr = self.socket.accept()
            print("Connected by: ", str(self.addr))
        else:
            self.soc = self.socket
            while True:
                try:
                    self.soc.connect((self.ip, self.port))
                    break
                except ConnectionRefusedError:
                    self.soc.close()
                    print("Server is not found.\nTry 2s later.")
                    time.sleep(2)
    
    def send(self, data:str):
        self.soc.send(data.encode())
        
    def recv(self)->str:
        data = self.soc.recv(1024)
        return data.decode()
    
    def close(self):
        self.soc.close()
        self.socket.close()
        
if __name__ == "__main__":
    com = TcpCommunicator(ip="192.168.1.228", port=50001, hosting=True)
    com.connect()
    
    while True:
        tx = input("Input data: ")
        com.send(tx)
        rx = com.recv()
        print(f"Received data: {rx}")
        
        if tx == "q" or rx == "q":
            com.close()
            break