"""
Terminal component using NiceGUI's built-in xterm with Docker container support
"""
from nicegui import ui
import asyncio
import subprocess
import os
import select
import fcntl
import pty
import struct
import termios
import signal

class Terminal:
    def __init__(self):
        self.master_fd = None
        self.slave_fd = None
        self.process = None
        self.process_task = None
        self.xterm = None
        self.container_name = None
        self.shell_pid = None
        
    def create(self, on_close=None):
        with ui.card().classes('w-full p-0'):
            if on_close:
                with ui.row().classes('w-full justify-end p-2 pb-0'):
                    ui.button(icon='close', on_click=lambda: self._handle_close(on_close)).props('flat dense round size=sm color=red')
            
            self.xterm = ui.xterm({
                'cols': 180, 
                'rows': 20,
                'theme': {
                    'background': '#0a0e12',
                    'foreground': '#d1d5db',
                    'cursor': '#14b8a6',
                    'cursorAccent': '#0a0e12',
                    'selection': '#14b8a633',
                    'black': '#1a1d24',
                    'red': '#ff5555',
                    'green': '#51cf66',
                    'yellow': '#fbbf24',
                    'blue': '#4dabf7',
                    'magenta': '#cc5de8',
                    'cyan': '#14b8a6',
                    'white': '#e5e7eb',
                    'brightBlack': '#6c757d',
                    'brightRed': '#ff6b6b',
                    'brightGreen': '#69db7c',
                    'brightYellow': '#ffd43b',
                    'brightBlue': '#74c0fc',
                    'brightMagenta': '#da77f2',
                    'brightCyan': '#3bc9db',
                    'brightWhite': '#f8f9fa'
                },
                'fontFamily': '"Fira Code", "Cascadia Code", "Consolas", "Courier New", monospace',
                'fontSize': 14,
                'fontWeight': 400,
                'fontWeightBold': 700,
                'lineHeight': 1.2,
                'letterSpacing': 0,
                'cursorBlink': True,
                'cursorStyle': 'block',
                'scrollback': 1000,
                'tabStopWidth': 4
            }).classes('w-full').style('height: 400px; border-radius: 0 0 8px 8px;')
            self.xterm.on_data(self._handle_input)
    
    def _handle_close(self, on_close):
        self.stop()
        on_close()
    
    async def start_docker_shell(self, container_name: str, image: str = 'nodearena-intro:latest', create_new: bool = True):
        """Start a shell inside a Docker container using PTY with proper terminal emulation
        
        Args:
            container_name: Name of the container
            image: Docker image to use
            create_new: If True, create a new container. If False, attach to existing container.
        """
        self.container_name = container_name
        
        if create_new:
            try:
                subprocess.run(
                    ['docker', 'rm', '-f', container_name],
                    capture_output=True,
                    timeout=5
                )
                await asyncio.sleep(0.2)
            except:
                pass
            
            start_cmd = [
                'docker', 'run',
                '-d',
                '-it',
                '--name', container_name,
                image,
                '/bin/bash', '-c', 'while true; do sleep 1; done'
            ]
            
            try:
                result = subprocess.run(start_cmd, capture_output=True, text=True, timeout=5)
                if result.returncode != 0:
                    self.xterm.write(f'Error starting container: {result.stderr}\r\n')
                    return
                
                await asyncio.sleep(0.5)
            except Exception as e:
                self.xterm.write(f'Error: {str(e)}\r\n')
                return
        else:
            try:
                result = subprocess.run(
                    ['docker', 'inspect', '-f', '{{.State.Running}}', container_name],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode != 0 or result.stdout.strip() != 'true':
                    self.xterm.write(f'Container {container_name} is not running\r\n')
                    return
            except Exception as e:
                self.xterm.write(f'Error checking container: {str(e)}\r\n')
                return
        
        try:
            self.master_fd, self.slave_fd = pty.openpty()
            self._set_winsize(self.master_fd, 22, 180)
            
            self.shell_pid = os.fork()
            
            if self.shell_pid == 0:
                os.setsid()
                os.dup2(self.slave_fd, 0)
                os.dup2(self.slave_fd, 1)
                os.dup2(self.slave_fd, 2)
                os.close(self.master_fd)
                os.close(self.slave_fd)
                
                os.environ['TERM'] = 'xterm-256color'
                os.environ['COLUMNS'] = '180'
                os.environ['LINES'] = '22'
                
                os.execvp('docker', ['docker', 'exec', '-it', container_name, '/bin/bash'])
            else:
                os.close(self.slave_fd)
                self.slave_fd = None
                fcntl.fcntl(self.master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
                self.process_task = asyncio.create_task(self._read_output())
                
        except Exception as e:
            self.xterm.write(f'Error: {str(e)}\r\n')
            if create_new:
                self.cleanup_container()
    
    async def start_local_shell(self, shell: str = '/bin/bash'):
        """Start a local shell with proper PTY"""
        self.master_fd, self.slave_fd = pty.openpty()
        self._set_winsize(self.master_fd, 22, 180)
        
        self.shell_pid = os.fork()
        
        if self.shell_pid == 0:
            os.setsid()
            os.dup2(self.slave_fd, 0)
            os.dup2(self.slave_fd, 1)
            os.dup2(self.slave_fd, 2)
            os.close(self.master_fd)
            os.close(self.slave_fd)
            
            os.environ['TERM'] = 'xterm-256color'
            os.environ['COLUMNS'] = '180'
            os.environ['LINES'] = '22'
            
            os.execvp(shell, [shell])
        else:
            os.close(self.slave_fd)
            self.slave_fd = None
            fcntl.fcntl(self.master_fd, fcntl.F_SETFL, os.O_NONBLOCK)
            self.process_task = asyncio.create_task(self._read_output())
    
    def _set_winsize(self, fd, rows, cols):
        winsize = struct.pack('HHHH', rows, cols, 0, 0)
        fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)
    
    def _handle_input(self, e):
        if self.master_fd:
            try:
                os.write(self.master_fd, e.data.encode('utf-8'))
            except (OSError, BrokenPipeError):
                pass
    
    async def _read_output(self):
        while self.master_fd:
            try:
                await asyncio.sleep(0.01)
                
                readable, _, _ = select.select([self.master_fd], [], [], 0)
                if readable:
                    try:
                        output = os.read(self.master_fd, 4096)
                        if output:
                            decoded = output.decode('utf-8', errors='replace')
                            self.xterm.write(decoded)
                        else:
                            break
                    except OSError:
                        break
                
                if self.shell_pid:
                    try:
                        pid, status = os.waitpid(self.shell_pid, os.WNOHANG)
                        if pid != 0:
                            break
                    except ChildProcessError:
                        break
                        
            except Exception as e:
                print(f"Terminal error: {e}")
                break
        
        self.xterm.write('\r\n\n[Session ended]\r\n')
    
    def cleanup_container(self):
        if self.container_name:
            try:
                subprocess.run(
                    ['docker', 'rm', '-f', self.container_name],
                    timeout=5,
                    capture_output=True
                )
            except:
                pass
    
    def stop(self):
        if self.process_task:
            self.process_task.cancel()
        
        if self.shell_pid:
            try:
                os.kill(self.shell_pid, signal.SIGTERM)
                os.waitpid(self.shell_pid, 0)
            except:
                pass
            self.shell_pid = None
        
        if self.master_fd:
            try:
                os.close(self.master_fd)
            except:
                pass
            self.master_fd = None
        
        if self.slave_fd:
            try:
                os.close(self.slave_fd)
            except:
                pass
            self.slave_fd = None
    
    def force_cleanup(self):
        self.stop()
        self.cleanup_container()
        self.container_name = None

def create_terminal(use_docker: bool = False, 
                   container_name: str = None, 
                   image: str = 'nodearena-intro:latest',
                   create_new_container: bool = True,
                   on_close=None) -> Terminal:
    terminal = Terminal()
    terminal.create(on_close=on_close)
    
    if use_docker:
        if not container_name:
            raise ValueError("container_name is required when use_docker=True")
        asyncio.create_task(terminal.start_docker_shell(container_name, image, create_new_container))
    else:
        asyncio.create_task(terminal.start_local_shell())
    
    return terminal