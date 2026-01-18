import socket
import time


class TcpCommunicator():
    def __init__(self, ip:str="localhost", port:int=50001, hosting:bool=False, data_num:int=2, seperator:str="_"):
        self.ip = ip
        self.port = port
        self.host = hosting
        self.data_num = data_num
        self.seperator = seperator
        
        self._communication_cycle = 0.02
        
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._status = False
        self.data_recv = b""
        self.correct_data = "0" + f"{seperator}0" * (data_num - 1)
        
    def connect(self):
        if self.host:
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        self._status = True
    
    def send(self, data:list, starts_with='s'):
        if len(data) != 2:
            print(f"Expected {self.data_num} datas but {len(list)} were given.")
            return
        final_data = starts_with + str(data.pop(0))
        for additional_data in data:
            final_data += self.seperator + str(additional_data)
        self.soc.send(final_data.encode())
        
    def recv(self) -> str:
        self.data_recv = self.soc.recv(1024)
        return self.data_recv.decode()
    
    def loop_recv(self):
        while self._status:
            d = self.soc.recv(1024)
            if d != b"":
                self.data_recv = d
    
    def loop_send(self, reader):
        while self._status:
            time.sleep(self._communication_cycle)
            self.send(data=[reader.roll, reader.pitch], starts_with='s')
    
    def get_data(self, starts_with='s') -> str:
        d = self.data_recv.decode()
        index = d.rfind(starts_with)
        if index != -1:
            self.correct_data = d[index+1:]
        return self.correct_data
    
    def close(self):
        self._status = False
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