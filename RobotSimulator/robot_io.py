import threading
import time
import struct
import socketserver
#----------------------#
ROBOT_PORT = 5022

ROBOT_INPUTS = [              # FC 2 Read Discrete Inputs  -> PLC
    "ESTOP_OK",               # 0  %IX0.0  1 = E-stop NOT pressed
    "DOOR_CLOSED",            # 1  %IX0.1  1 = guard door closed
    "PB_START",               # 2  %IX0.2  local panel
    "PB_STOP",                # 3  %IX0.3  local panel
    "PB_RESET",               # 4  %IX0.4  local panel
    "SEL_REMOTE",             # 5  %IX0.5  1 = SCADA, 0 = local panel
    "PRODUCT_AT_PICK",        # 6  %IX0.6
    "CAMERA_DONE",            # 7  %IX0.7
    "CAMERA_PASS",            # 8  %IX1.0
    "ROBOT_READY",            # 9  %IX1.1
    "ROBOT_BUSY",             # 10 %IX1.2
    "ROBOT_DONE",             # 11 %IX1.3
    "ROBOT_AT_HOME",          # 12 %IX1.4
    "ROBOT_FAULT",            # 13 %IX1.5
    "VACUUM_OK",              # 14 %IX1.6
    "CARTON_AT_PACK",         # 15 %IX1.7
    "CARTON_OUT",             # 16 %IX2.0  0.5 s pulse
]

ROBOT_REGISTERS = [           # FC 4 Read Input Registers -> PLC
    "ROBOT_POS",              # 0 %IW0  position number
    "ROBOT_ANGLE",            # 1 %IW1  degrees
    "ROBOT_REACH",            # 2 %IW2  mm
    "CAMERA_CODE",            # 3 %IW3  0 good, 1 dent, 2 no label, 3 wrong colour
]

ROBOT_COILS = [               # FC 15 Write Multiple Coils <- PLC
    "INFEED_RUN",             # 0  %QX0.0
    "OUTFEED_RUN",            # 1  %QX0.1
    "CAMERA_TRIGGER",         # 2  %QX0.2
    "ROBOT_ENABLE",           # 3  %QX0.3
    "ROBOT_GO",               # 4  %QX0.4
    "ROBOT_RESET",            # 5  %QX0.5
    "VACUUM_ON",              # 6  %QX0.6
    "LAMP_GREEN",             # 7  %QX0.7
    "LAMP_AMBER",             # 8  %QX1.0
    "LAMP_RED",               # 9  %QX1.1
    "BUZZER",                 # 10 %QX1.2
]

ROBOT_HOLDING = [             # FC 16 / FC 6 Write Registers <- PLC   (NEW)
    "ROBOT_TARGET",           # 0 %QW0  0 home, 1 pick, 11-19 slot, 20 reject
    "ROBOT_SPEED",            # 1 %QW1  10-100 %
    "PATTERN",                # 2 %QW2  4, 6 or 9 per carton
]

COMMS_TIMEOUT = 2.0   # seconds without a PLC request before we say "not connected"

def pack_bits(bits):
    """Pack a list of True/False into bytes: 8 bits per byte, bit 0 first (Modbus order)."""
    data = bytearray((len(bits) + 7) // 8)
    for i, bit in enumerate(bits):
        if bit:
            data[i // 8] |= 1 << (i % 8)
    return bytes(data)


class Station:
    """One remote I/O module, like TankIO or BottleIO in OpenPLC."""

    def __init__(self, name, port, input_names, coil_names, register_names, holding_names):
        self.name = name
        self.port = port
        self.input_names = input_names
        self.coil_names = coil_names
        self.register_names = register_names
        self.holding_names = holding_names

        # Simulator -> PLC  (the PLC reads these)
        self.inputs = {}
        for n in input_names:
            self.inputs[n] = False
        self.registers = {}
        for n in register_names:
            self.registers[n] = 0

        # PLC -> simulator  (the PLC writes these)
        self.coils = {}
        for n in coil_names:
            self.coils[n] = False
        self.holding = {}
        for n in holding_names:
            self.holding[n] = 0

        self.lock = threading.Lock()
        self.last_request = None       # clock time of the PLC's last message
        self.requests = 0              # how many messages the PLC has sent

    def connected(self):
        """True if the PLC has talked to us in the last 2 seconds."""
        if self.last_request is None:
            return False
        return time.monotonic() - self.last_request < COMMS_TIMEOUT

    def answer(self, request):
        """Answer one Modbus request from the PLC and return the reply."""
        function = request[0]
        with self.lock:
            self.last_request = time.monotonic()
            self.requests += 1

            if function == 2:                                  # FC 2: Read Discrete Inputs
                start, count = struct.unpack(">HH", request[1:5])
                if start + count > len(self.input_names):
                    return bytes([function + 0x80, 2])         # error 2: no such address
                bits = []
                for name in self.input_names[start:start + count]:
                    bits.append(self.inputs[name])
                data = pack_bits(bits)
                return bytes([function, len(data)]) + data

            if function == 4:                                  # FC 4: Read Input Registers
                start, count = struct.unpack(">HH", request[1:5])
                if start + count > len(self.register_names):
                    return bytes([function + 0x80, 2])
                data = b""
                for name in self.register_names[start:start + count]:
                    value = max(0, min(65535, int(self.registers[name])))
                    data += struct.pack(">H", value)
                return bytes([function, len(data)]) + data

            if function == 15:                                 # FC 15: Write Multiple Coils
                start, count, size = struct.unpack(">HHB", request[1:6])
                if start + count > len(self.coil_names):
                    return bytes([function + 0x80, 2])
                data = request[6:6 + size]
                for i in range(count):
                    name = self.coil_names[start + i]
                    self.coils[name] = bool(data[i // 8] >> (i % 8) & 1)
                return request[0:5]

            if function == 16:

                start, count, size = struct.unpack(">HHB", request[1:6])
                if start + count > len(self.holding_names):
                    return bytes([function + 0x80, 2])
                for i in range(count):
                    name = self.holding_names[start + i]
                    value = struct.unpack(">H", request[6 + i * 2:8 + i * 2])[0]
                    self.holding[name] = value
                return request[0:5]
            
            if function == 6:

                address, value = struct.unpack(">HH", request[1:5])
                if address >= len(self.holding_names):
                    return bytes([function + 0x80, 2])
                self.holding[self.holding_names[address]] = value
                return request[0:5]
       
        return bytes([function + 0x80, 1])                     # error 1: function not supported

        

        
                             

    
    def start(self):                                           # ← now inside Station
        """Open this station's port and answer the PLC in the background."""
        server = socketserver.ThreadingTCPServer(("0.0.0.0", self.port), ModbusHandler)
        server.daemon_threads = True
        server.station = self
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(self.name, "is listening on port", self.port)
        



def make_station():
    """Create the robot cell: one remote I/O module on port 5022."""
    return Station("Robot cell", ROBOT_PORT, ROBOT_INPUTS, ROBOT_COILS,
                   ROBOT_REGISTERS, ROBOT_HOLDING)


#---- Step 4 : Modbus TCP --------#

def recv_exact(sock, size):
    """Receive exactly size bytes from the PLC, or None if the PLC hung up."""
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data



class ModbusHandler(socketserver.BaseRequestHandler):
    """Talks to one connected PLC: read a requestm answer it, repeat."""

    def handle(self):
        station = self.server.station
        try:
            while True:
                header = recv_exact(self.request, 7)
                if header is None:
                    return
                transaction, protocol, length, unit = struct.unpack(">HHHB", header)
                if length < 2:
                    return
                request = recv_exact(self.request, length - 1)
                if request is None:
                    return
                reply = station.answer(request)
                header = struct.pack(">HHHB", transaction, protocol, len(reply) + 1, unit)
                self.request.sendall(header + reply)
        except OSError:
            return
        
    


# ---------- Self-check: wait for the PLC (or a test PLC) ----------
if __name__ == "__main__":
    robot = make_station()
    robot.start()

    # How the cell looks before anyone touches it
    robot.inputs["ESTOP_OK"] = True          # E-stop is NOT pressed
    robot.inputs["DOOR_CLOSED"] = True       # guard door is closed
    robot.inputs["ROBOT_AT_HOME"] = True     # robot starts at Home
    robot.inputs["CARTON_AT_PACK"] = True    # an empty carton is in place
    robot.registers["ROBOT_ANGLE"] = 90      # robot arm points at 90 degrees

    print("Waiting for the PLC... press Ctrl + C to stop.")
    while True:
        time.sleep(1)
        on = []
        for name in robot.coil_names:
            if robot.coils[name]:
                on.append(name)
        print("PLC connected:", robot.connected(),
              "| requests:", robot.requests,
              "| coils ON:", on,
              "| holding:", robot.holding)


