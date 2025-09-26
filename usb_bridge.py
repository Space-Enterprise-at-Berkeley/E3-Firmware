import serial

# stream setup
ser = serial.Serial('/dev/cu.SLAB_USBtoUART') #this port is only for testing
ser.baudrate = 921600
ser.flushInput()
ser.flushOutput()
count = 1


def compute_packet_checksum(packet_bytes: list[int]) -> int:
    """
    Compute checksum in the same way as the C code.

    packet_bytes layout:
    [ id,
      len,
      timestamp0, timestamp1, timestamp2, timestamp3,
      data... ]
    """
    if len(packet_bytes) < 6:
        return -1 # lost packet?

    pkt_id   = packet_bytes[0]
    pkt_len  = packet_bytes[1]
    ts_bytes = packet_bytes[2:6]
    data     = packet_bytes[6:6 + pkt_len]   # only the first 'len' bytes count

    sum1 = 0
    sum2 = 0

    # id
    sum1 = (sum1 + pkt_id) & 0xFF
    sum2 = (sum2 + sum1)   & 0xFF

    # len
    sum1 = (sum1 + pkt_len) & 0xFF
    sum2 = (sum2 + sum1)    & 0xFF

    # timestamp[4]
    for b in ts_bytes:
        sum1 = (sum1 + b) & 0xFF
        sum2 = (sum2 + sum1) & 0xFF

    # data[0:len]
    for b in data:
        sum1 = (sum1 + b) & 0xFF
        sum2 = (sum2 + sum1) & 0xFF

    # final 16-bit value: high byte = sum2, low byte = sum1
    return (sum2 << 8) | sum1

def verify_packet(packet_bytes: list[int]) -> bool:
    """
    Verify the packet checksum.

    packet_bytes layout:
    [ id,
      len,
      timestamp0, timestamp1, timestamp2, timestamp3,
      checksum0, checksum1,        # <-- expected checksum (low, high)
      data... ]
    """
    if len(packet_bytes) < 8:
        return False # lost packet?

    # Extract fields
    pkt_id   = packet_bytes[0]
    pkt_len  = packet_bytes[1]
    ts_bytes = packet_bytes[2:6]
    checksum_bytes = packet_bytes[6:8]       # [low, high]
    data     = packet_bytes[8:8 + pkt_len]

    # Reconstruct the packet *without* its stored checksum bytes,
    # matching the original C code's input to computePacketChecksum.
    packet_for_checksum = [pkt_id, pkt_len] + ts_bytes + data

    csum = compute_packet_checksum(packet_for_checksum)

    low  = csum & 0xFF
    high = (csum >> 8) & 0xFF

    return low == checksum_bytes[0] and high == checksum_bytes[1]

while True:
    line = ser.read_until(b']]') #end of packet delimeter
    #print(list(line))

    if b'[[' in line:
        packet = list(line[line.find(b'[[')+2:-2]) #trims off the delimeters
        if verify_packet(packet): # checksum works
            print(packet)
        else: 
            print("checksum fail")
    else:
        print("invalid packet")