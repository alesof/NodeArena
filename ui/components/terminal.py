"""
Terminal component using NiceGUI's built-in xterm
"""
from nicegui import ui
import asyncio
import pty
import os
import select
import struct
import fcntl
import termios
import signal


class Terminal:
    def __init__(self):
        self.master_fd = None
        self.slave_fd = None
        self.process_task = None
        self.xterm = None
        self.shell_pid = None
        
    def create(self):
        with ui.card().classes('w-full p-0'):
            self.xterm = ui.xterm({'cols': 180, 'rows': 22}).classes('w-full').style('height: 400px;')
            self.xterm.on_data(self._handle_input)
    
    async def start_shell(self, shell: str = '/bin/bash'):
        self.master_fd, self.slave_fd = pty.openpty()
        
        self._set_winsize(self.master_fd, 20, 180)
        
        self.shell_pid = os.fork()
        if self.shell_pid == 0:
            os.setsid()
            os.dup2(self.slave_fd, 0)
            os.dup2(self.slave_fd, 1)
            os.dup2(self.slave_fd, 2)
            os.close(self.master_fd)
            os.close(self.slave_fd)
            
            os.environ['COLUMNS'] = '180'
            os.environ['LINES'] = '20'
            
            os.execvp(shell, [shell])
        else:
            os.close(self.slave_fd)
            fcntl.fcntl(self.master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
            
            self.process_task = asyncio.create_task(self._read_output())
    
    def _set_winsize(self, fd, rows, cols):
        winsize = struct.pack('HHHH', rows, cols, 0, 0)
        fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)
    
    def _handle_input(self, e):
        if self.master_fd:
            try:
                os.write(self.master_fd, e.data.encode('utf-8'))
            except OSError:
                pass
    
    async def _read_output(self):
        while self.master_fd:
            try:
                await asyncio.sleep(0.01)
                
                readable, _, _ = select.select([self.master_fd], [], [], 0)
                if readable:
                    try:
                        output = os.read(self.master_fd, 1024)
                        if output:
                            decoded = output.decode('utf-8', errors='replace')
                            self.xterm.write(decoded)
                    except OSError:
                        break
            except Exception as e:
                print(f"Terminal error: {e}")
                break
    
    def stop(self):
        if self.process_task:
            self.process_task.cancel()
        if self.master_fd:
            try:
                os.close(self.master_fd)
            except:
                pass
            self.master_fd = None


def create_terminal() -> Terminal:
    terminal = Terminal()
    terminal.create()
    return terminal