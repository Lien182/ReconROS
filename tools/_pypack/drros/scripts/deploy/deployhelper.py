import os
import time
import sys
import re
import paramiko
import threading
from stat import *

import threading, paramiko

strdata=''
fulldata=''

sem = threading.Semaphore()


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    WHITE = '\033[97m'


class ssh:
    shell = None
    client = None
    transport = None

    def __init__(self, address, username, password, id):
        self.client = paramiko.client.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.client.AutoAddPolicy())
        self.client.connect(address, username=username, password=password, look_for_keys=False)
        self.transport = paramiko.Transport((address, 22))
        self.transport.connect(username=username, password=password)
        self.id = id
        thread = threading.Thread(target=self.process)
        thread.daemon = True
        thread.start()

    def close_connection(self):
        if(self.client != None):
            self.client.close()
            self.transport.close()

    def open_shell(self):
        self.shell = self.client.invoke_shell()

    def send_shell(self, command):
        if(self.shell):
            self.shell.send(command + "\n")
        else:
            print("Shell not opened.")

    def process(self):
        global strdata, fulldata
        while True:
            # Print data when available
            if self.shell is not None and self.shell.recv_ready():
                alldata = self.shell.recv(512)
                while self.shell.recv_ready():
                    alldata += self.shell.recv(512)
                
                sem.acquire()
                print(bcolors.FAIL + str(self.id))
                print(bcolors.WHITE + str(alldata,'utf-8'))
                sem.release()
                #strdata = strdata + str(alldata)
                #fulldata = fulldata + str(alldata)
                #strdata = self.print_lines(strdata) # print all received data except last line
            else:
                time.sleep(0.1)

    def print_lines(self, data):
        last_line = data
        if '\n' in data:
            lines = data.splitlines()
            for i in range(0, len(lines)-1):
                print(lines[i])
            last_line = lines[len(lines) - 1]
            if data.endswith('\n'):
                print(last_line)
                last_line = ''
        return last_line

    def sendfile(self, file, dest,exec):
        sftp = self.client.open_sftp()
        sftp.put(file, dest)
        print("Copy file: " + file + " " + dest)
        if exec == 1:
            sftp.chmod("/mnt/recobop", S_IEXEC)
        sftp.close()


sshUsername = "root"
sshPassword = "xilinx"


class ReconROSClient:
    _id = 0
    _name = ""

    def __init__(self, fname, id):
        self.id = id
        self.name=fname
        self.ssh = ssh(self.get_ip(), sshUsername, sshPassword, self.id)
        self.thread = threading.Thread(target=self.thread_function, args=(0,))
        self.ExitRequest = False
        print("New ReconROS client: "+ fname )
    def start(self):
            
        self.thread.start()

    def join(self):
        self.thread.join()

    def get_ip(self):
        return "192.168.2." + self.id




    def disconnect(self):
        self.ExitRequest = True
       
    


    def thread_function(self,c):
        #self.connect()
        self.ssh.sendfile(self.name + "/build.hw/myReconOS.runs/impl_1/design_1_wrapper.bit", "/mnt/download.bit", False)
        self.ssh.sendfile(self.name + "/build.sw/recobop", "/mnt/recobop", True)

        self.ssh.open_shell()
        self.ssh.send_shell('cat /mnt/download.bit > /dev/xdevcfg')
        self.ssh.send_shell('cd /opt/reconos/')
        self.ssh.send_shell('./reconos_init.sh')
        self.ssh.send_shell('source /opt/ros/dashing/setup.bash')
        self.ssh.send_shell('source /mnt/msg_pack/install/local_setup.bash')
        self.ssh.send_shell('/mnt/recobop')
        
        while not self.ExitRequest:
            time.sleep(1)
 
        self.disconnect()



subfolders = [ f.path for f in os.scandir('.') if f.is_dir() ]
clients = []

for f in subfolders:
    if re.search('(?<=./zed)_[0-9]', f):        
        id =re.search('(?<=./zed_)[0-9][0-9]', f)[0]
        clients.append(ReconROSClient(f,id))

threads = []

for c in clients:
    print(c.name)
    c.start()


try:
    while True:
        time.sleep(10)
except KeyboardInterrupt:
    pass


for c in clients:
    print(c.name)
    c.disconnect()


for t in clients:
    c.join()




