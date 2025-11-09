import serial, threading, queue, json, time

class SerialWorker:
    def __init__(self, port, baudrate=115200, debug_print=True):
        self.ser: serial.Serial = None
        
        self.port = port
        self.baudrate = baudrate
        
        self.debug_print = debug_print
        
        self.queue = queue.Queue()
        self._run = False

    def open(self):
        if not self._run:
            self._run = True
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            if self.debug_print:
                print(f"[SerialWorker]: Connected to {self.port} @ {self.baudrate}")

            threading.Thread(target=self._reader, daemon=True).start()
        else:
            if self.debug_print:
                print(f"[SerialWorker]: Already connected to {self.port}")
    
    def close(self):
        self._run = False
        time.sleep(0.5) # wait reader thread to exit
        if self.ser and self.ser.is_open:
            self.ser.close()
            if self.debug_print:
                print(f"[SerialWorker]: Disconnected from {self.port}")
        else:
            if self.debug_print:
                print(f"[SerialWorker]: Serial port already closed or not initialized")
    
    def _check_is_json(self, text: str) -> bool:
        if not text or text[0] not in "{[":
            return False
        try:
            json.loads(text)
            return True
        except json.JSONDecodeError:
            return False

    def _reader(self):
        buffer = ""
        while self._run:
            data = self.ser.read(self.ser.in_waiting or 1).decode(errors="ignore")
            if not data:
                continue
            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if self._check_is_json(line.strip()):
                    self.queue.put(json.loads(line.strip()))
                elif self.debug_print:
                    print(f"[SerialWorker] receive line: <{line.strip()}>")

    def send_command(self, msg: str):
        if '\n' in msg:
            msg = msg.replace("\n", "")
        self.ser.write((msg + "\n").encode())

    def read_response(self, timeout=0.1):
        try:
            return self.queue.get(timeout=timeout)
        except queue.Empty:
            return None
        

if __name__ == "__main__":
    import time
    
    last_fps_time = time.time()
    frame_count = 0
    
    COM = "COM18"
    anchor_id = 0xff00 # now on serial port
    tag_id = 0x0000
    
    worker = SerialWorker(port=COM, baudrate=115200, debug_print=True)
    worker.open() 
    
    try:
        while True:
            worker.send_command(f"ping {tag_id}")
            ping_response = worker.read_response()
            if ping_response:
                pass
                # print("✅ ping success:", ping_response)
            else:
                print("❌ ping timeout")

            # if print success start range test, trigger the tag to start ranging with self
            worker.send_command(f"trigger {tag_id} {anchor_id}")
            range_response = worker.read_response()
            if range_response:
                print("📡 range report:", range_response)
            else:
                print("❌ range timeout")
            
            # calc the fps
            frame_count += 1
            if time.time() - last_fps_time >= 1.0:
                fps = frame_count / (time.time() - last_fps_time)
                print(f"🔄 FPS: {fps:.2f}")
                frame_count = 0
                last_fps_time = time.time()

    except KeyboardInterrupt:
        pass
        
    worker.close()
    
    
    
'''

🔄 FPS: 16.19
✅ ping success: {'event': 'ping_resp', 'node_id': 0, 'system_state': 135, 'voltage_mv': 3950}
📡 range report: {'event': 'range_final', 'node_a_id': 0, 'node_b_id': 65280, 'distance_m': 0.0, 'rssi_dbm': -63.23}
✅ ping success: {'event': 'ping_resp', 'node_id': 0, 'system_state': 188, 'voltage_mv': 3950}
📡 range report: {'event': 'range_final', 'node_a_id': 0, 'node_b_id': 65280, 'distance_m': 0.0, 'rssi_dbm': -63.25}
✅ ping success: {'event': 'ping_resp', 'node_id': 0, 'system_state': 241, 'voltage_mv': 3950}
📡 range report: {'event': 'range_final', 'node_a_id': 0, 'node_b_id': 65280, 'distance_m': 0.0, 'rssi_dbm': -63.23}
✅ ping success: {'event': 'ping_resp', 'node_id': 0, 'system_state': 38, 'voltage_mv': 3950}
📡 range report: {'event': 'range_final', 'node_a_id': 0, 'node_b_id': 65280, 'distance_m': 0.0, 'rssi_dbm': -63.86}
✅ ping success: {'event': 'ping_resp', 'node_id': 0, 'system_state': 91, 'voltage_mv': 3950}
📡 range report: {'event': 'range_final', 'node_a_id': 0, 'node_b_id': 65280, 'distance_m': 0.0, 'rssi_dbm': -64.2}

'''