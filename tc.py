import threading
import time
import websockets
import asyncio
import json

# ==================== WebSocket Classes ====================
class CommandHandler(threading.Thread):
    def __init__(self, ws_uri='ws://base-station:8000/commands'):
        super().__init__()
        self.daemon = True
        self.ws_uri = ws_uri
        self.current_command = None
        self.lock = threading.Lock()
        self.running = True

    async def _ws_listener(self):
        async with websockets.connect(self.ws_uri) as ws:
            while self.running:
                try:
                    message = await ws.recv()
                    with self.lock:
                        self.current_command = json.loads(message).get('command')
                except Exception as e:
                    print(f"Command WS error: {e}")
                    await asyncio.sleep(1)

    def get_command(self):
        with self.lock:
            cmd = self.current_command
            self.current_command = None  # Clear after reading
            return cmd

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        while self.running:
            try:
                loop.run_until_complete(self._ws_listener())
            except Exception as e:
                print(f"Command handler restarting: {e}")
                time.sleep(1)

    def stop(self):
        self.running = False


class UserTracker(threading.Thread):
    def __init__(self, ws_uri='ws://base-station:8000/tracking'):
        super().__init__()
        self.daemon = True
        self.ws_uri = ws_uri
        self.node_position = {'x': 0, 'y': 0, 'z': 0}
        self.frame_center = (320, 240)  # Example HD frame size
        self.lock = threading.Lock()
        self.running = True

    async def _ws_listener(self):
        async with websockets.connect(self.ws_uri) as ws:
            while self.running:
                try:
                    message = await ws.recv()
                    with self.lock:
                        self.node_position = json.loads(message)
                except Exception as e:
                    print(f"Tracking WS error: {e}")
                    await asyncio.sleep(1)

    def get_position(self):
        with self.lock:
            return self.node_position.copy()

    def run(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        while self.running:
            try:
                loop.run_until_complete(self._ws_listener())
            except Exception as e:
                print(f"User tracker restarting: {e}")
                time.sleep(1)

    def stop(self):
        self.running = False
