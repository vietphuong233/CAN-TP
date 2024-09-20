""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Import                                                                       "
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
import can
import threading
import time
from enum import Enum

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Definition                                                                   "
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
# Define constants
CAN_ID = 0x123
TIMEOUT = 5
BLOCK_SIZE = 6
MAX_WAIT_COUNT = 3

# Flow Control Frame Types
class FlowControlType(Enum):
    CTS      = 0x00
    WAIT     = 0x01
    OVERFLOW = 0x02


""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Code                                                                         "
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
class CAN_TP:
    def __init__(self, channel=1, bitrate=500000, is_fd=False, dlc=8):
        self.bus = can.interface.Bus(channel=channel, interface='neovi', bitrate=bitrate,  receive_own_messages=False)

        # Message attribute
        self.dlc = dlc
        self.is_fd = is_fd

        # Timeout attribute
        self.timeout = TIMEOUT
        self.STmin = 250

        # Wait attributes
        self.wait_count = 0

        # Block size attribute
        self.original_block_size = BLOCK_SIZE
        self.block_size = self.original_block_size

        # Timing Parameters
        self.N_As = 3000
        self.N_Ar = 3000
        self.N_Bs = 3000
        self.N_Br = 3000
        self.N_Cs = 3000
        self.N_Cr = 3000

        # Timers initialize
        self.N_As_timer = None
        self.N_Ar_timer = None
        self.N_Bs_timer = None
        self.N_Br_timer = None
        self.N_Cs_timer = None
        self.N_Cr_timer = None

        # Timeout flag
        self.timeout_occurred = False

        # Data storage
        self.max_data_size = self.dlc - 1
        self.sent_data_length = 0
        self.first_frame_received = False
        self.received_data = bytearray()

    ############################################################################
    # Timer Methods                                                            #
    ############################################################################
    def start_timer(self, timer_name, timeout, callback):
        """Start a timer with given timeout and callback"""
        if timer_name is not None:
            timer_name.cancel() # Cancel any previous timer
        timer = threading.Timer(timeout/1000.0, callback)
        timer.start()
        return timer

    def stop_timer(self, timer:threading.Timer):
        """Stop a timer"""
        if timer is not None:
            timer.cancel()

    def handle_timeout(self, timer_name):
        """Handle timeout by setting flag and stop the thread"""
        self.timeout_occurred = True
        raise TimeoutError(f"{timer_name} timeout occured!")

    ############################################################################
    # Sender Methods                                                           #
    ############################################################################
    def send_message(self, data):
        """Send a CAN TP message, split into multiple frames if needed"""
        # Start N_As timer
        self.N_As_timer = self.start_timer(self.N_As_timer, self.N_As, lambda: self.handle_timeout("N_As"))

        if len(data) <= self.max_data_size:
            self._send_single_frame(data)
        else:
            self.sent_data_length = len(data)
            self._send_multi_frame(data)

    def _send_single_frame(self, data):
        """Send CANTP single frame message"""
        pci = bytearray([len(data) & 0x0F]) # PCI for single frame
        message = can.Message(arbitration_id=CAN_ID, data=pci+data, is_extended_id=False, is_fd=self.is_fd, dlc=self.dlc)

        self.bus.send(message)
        print(f"Sent single frame message: {data}")

        #Stop N_As timer after sent successfully
        self.stop_timer(self.N_As_timer)

    def _send_multi_frame(self, data):
        """Send CANTP multi-frame message"""
        # Calculate number of consecutive frames
        total_length = len(data)

        # Send first frame
        self._send_first_frame(data[:self.max_data_size])

        # Track how many bytes have already sent
        bytes_sent   = self.max_data_size - 1
        # Frame index for consecutive frames
        frame_index = 1

        # Send remaining consecutive frames
        while bytes_sent < total_length:
            # Check timeout flag
            if self.timeout_occurred:
                raise TimeoutError("Timeout occurred during transmission.")

            # Wait for a flow control frame
            self.N_Bs_timer = self.start_timer(self.N_Bs_timer, self.N_Bs, lambda: self.handle_timeout("N_Bs"))
            self._wait_for_flow_control()
            self.stop_timer(self.N_Bs_timer)

            # Send consecutive frames based on block size
            frames_sent = 0
            while bytes_sent < total_length:
                # Calculate the next data segment to send
                start = frame_index * self.max_data_size - 1
                end = start + self.max_data_size

                # Send Consecutive frame
                self.stop_timer(self.N_Cs_timer)
                self._send_consecutive_frame(data[start:end], frame_index)
                self.N_Cs_timer = self.start_timer(self.N_Cs_timer, self.N_Cs, lambda: self.handle_timeout("N_Cs"))

                # STmin separation
                time.sleep(self.STmin/1000.0)

                # Update counters
                bytes_sent += end - start
                frame_index += 1
                frames_sent += 1
                self.block_size -= 1

                # Check if the remaining block size is enough
                if self.block_size < 2:
                    break

                # Check if the transmission is completed
                if bytes_sent >= total_length:
                    self.stop_timer(self.N_Cs_timer)

    def _send_first_frame(self, data):
        """Send the first frame of a multi-frame message"""
        # Create the PCI header for the First Frame
        pci_byte1 = 0x10 | ((self.sent_data_length>>8) & 0x0F)
        pci_byte2 = self.sent_data_length & 0x00FF

        # Create the message data: 2 bytes PCI + data
        message_data = bytearray([pci_byte1, pci_byte2]) + data[:self.max_data_size-1]

        message = can.Message(arbitration_id=CAN_ID, data=message_data, is_extended_id=False,  is_fd=self.is_fd, dlc=self.dlc)

        self.bus.send(message)
        print(f"Sent first frame message: {data}")

        # Stop N_As timer if sent successfully
        self.stop_timer(self.N_As_timer)

    def _send_consecutive_frame(self, data, sequence_number):
        """Send a consecutive frame of a multi-frame message"""
        pci = bytearray([0x20 | (sequence_number & 0x0F)]) # PCI for consecutive frame
        message = can.Message(arbitration_id=CAN_ID, data=pci+data, is_extended_id=False, is_fd=self.is_fd, dlc=self.dlc)

        self.bus.send(message)
        print(f"Sent consecutive frame message: {data}")

    def _wait_for_flow_control(self):
        """Wait for a flow control frame and handle it"""
        start_time = time.time()
        while True:
            # Check timeout flag
            if self.timeout_occurred:
                raise TimeoutError("Timeout occurred during transmission.")

            message = self.bus.recv(timeout=self.STmin/1000.0)
            if message:
                if message.arbitration_id == CAN_ID:
                    fc_data = message.data
                    fc_type = FlowControlType(fc_data[0] & 0x0F)
                    block_size = fc_data[1]
                    self.STmin = fc_data[2]
                    print(f"Received FC frame: Type={fc_type}, BS={block_size}, WaitTime={self.STmin}")

                    if fc_type == FlowControlType.CTS:
                        self.block_size = block_size
                        return
                    elif fc_type == FlowControlType.WAIT:
                        continue
                    elif fc_type == FlowControlType.OVERFLOW:
                        raise RuntimeError("Receiver is overloadded.")
            else:
                print("No Flow Control received, waiting...")

    ############################################################################
    # Receiver Methods                                                         #
    ############################################################################
    def receive_message(self):
        """Receive CAN TP message, reassemble if multi-frame"""
        while True:
            # Check timeout flag
            if self.timeout_occurred:
                raise TimeoutError("Timeout occurred during reception")

            message = self.bus.recv(timeout=self.STmin/1000.0)
            if message:
                if message.arbitration_id != CAN_ID:
                    print(f"Ignoring unexpected message ID: {message.arbitration_id}")
                else:
                    # Start N_Br Timer
                    self.N_Br_timer = self.start_timer(self.N_Br_timer, self.N_Br, lambda: self.handle_timeout("N_Br"))

                    pci_byte = message.data[0]
                    # Single frame
                    if (pci_byte & 0xF0) == 0x00:
                        self._handle_single_frame(message.data)
                        break
                    # First Frame
                    elif (pci_byte & 0xF0) == 0x10:
                        self._handle_first_frame(message.data)
                    # Consecutive Frame PCI check
                    elif (pci_byte & 0xF0) == 0x20:
                        self._handle_consecutive_frame(message.data)

                    print(f"received data len: {len(self.received_data)}")
                    print(f"expected length: {self.expected_length}")

                    # If the message length is less than self.max_data_size, this is the last frame
                    if len(self.received_data) >= self.expected_length:
                        self.stop_timer(self.N_Br_timer)
                        break
            else:
                print(f"No message received, waiting...")

        # print(f"Received message: {self.received_data}")
        return self.received_data

    def _handle_single_frame(self, data):
        """Handle the Single Frame"""
        print(f"Handling Single Frame: {data}")
        # Extract the data from the single frame
        self.received_data.extend(data[1:]) # Skip the PCI byte

    def _handle_first_frame(self, data):
        """Handle the First Frame"""
        print(f"Handling First Frame: {data}")
        self.first_frame_received = True
        self.expected_length = ((data[0]<<8)&0x0F00) | data[1] # Extract the expected length from PCI
        self.received_data.extend(data[2:]) # Skip PCI bytes

        # Simulate block size
        self.block_size -= 1

        # Send a Flow Control frame with Clear To Send status
        self._send_flow_control_frame(FlowControlType.CTS, self.block_size, self.STmin)

        # Set timer
        self.N_Cr_timer = self.start_timer(self.N_Cr_timer, self.N_Cr, lambda: self.handle_timeout("N_Cr"))

    def _handle_consecutive_frame(self, data):
        """Handle the Consecutive Frame"""
        sequence_number = data[0] & 0x0F
        print(f"Handling Consecutive Frame: {data}, Sequence Number: {sequence_number}")
        self.received_data.extend(data[1:]) # Skip the PCI byte


        # Simulate block size
        self.block_size -= 1

        # If block size is low, reset the block size and send another Flow Control Frame
        if self.block_size < 2:
            # Waiting request
            self.wait_count += 1
            self._send_flow_control_frame(FlowControlType.WAIT, self.block_size, self.STmin)
            # Simulate resquest block size from upper layers
            self.block_size = self.original_block_size
            time.sleep(self.STmin/1000.0)
            # Ready to receive
            self.wait_count = 0
            self._send_flow_control_frame(FlowControlType.CTS, self.block_size, self.STmin)
            # Set timer
            self.stop_timer(self.N_Cs_timer)
            self.N_Cr_timer = self.start_timer(self.N_Cr_timer, self.N_Cr, lambda: self.handle_timeout("N_Cr"))

    def _send_flow_control_frame(self, flow_control_type, block_size, wait_time):
        """Send a flow controll frame"""
        fc_data = bytearray([0x30 | flow_control_type.value, block_size, wait_time])
        message = can.Message(arbitration_id=CAN_ID, data=fc_data, is_extended_id=False, is_fd=self.is_fd, dlc=self.dlc)

        # Wait logic
        if flow_control_type == FlowControlType.WAIT:
            self.wait_count += 1
            if self.wait_count > MAX_WAIT_COUNT:
                print(f"Receiving Stopped! Maximum wait count reached: {MAX_WAIT_COUNT}")
                raise RuntimeError("Receiver exceeded maximum allowed wait FC frames.")

        # Send Flow Control Frame
        self.stop_timer(self.N_Br_timer)
        self.N_Ar_timer = self.start_timer(self.N_Ar_timer, self.N_Ar, lambda: self.handle_timeout("N_As"))
        self.bus.send(message)
        self.stop_timer(self.N_Ar_timer)

        print(f"Sent flow control frame: Type={flow_control_type}, BS={block_size}, WaitTime={wait_time}")

    def close(self):
        self.bus.shutdown()


""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" Test Code                                                                    "
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
if __name__ == "__main__":
    can_tp = CAN_TP(channel=1, bitrate=500000, is_fd=False, dlc= 8)
    try:
        # Un-comment to test sending
        test_data = bytearray("Introduction and functional overview\nThis specification defines the functionality, API and the configuration of the AUTOSAR\nBasic Software module CAN Transport Layer (CanTp).\nCanTp is the module between the PDU Router and the CAN Interface module.\n The main purpose of the CAN TP module is to segment and reassemble\nCAN I-PDUs longer than 8 bytes or longer than 64 bytes in case of CAN FD.".encode())
        can_tp.send_message(test_data)

        # Un-comment to test receiving
        # received_data = can_tp.receive_message()
        # if received_data:
        #     print(f"Received Data: {received_data.decode()}")
    finally:
        can_tp.close()

""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
" EOF                                                                          "
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
