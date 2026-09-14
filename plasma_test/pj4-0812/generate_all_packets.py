#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate all test packets with correct checksums
"""

import struct

def create_packet(emer_stop=0x00, vol_relay=0x00, vol_out=0, 
                 he_relay=0x00, he_out=0, ar_relay=0x00, ar_out=0):
    """Create data packet"""
    HEADER = 0xFEFF
    
    # Build first 12 bytes of data
    packet_data = struct.pack('<HBBHBHBH', 
                             HEADER,      # Header (little endian)
                             emer_stop,   # Emergency stop
                             vol_relay,   # Plasma power relay
                             vol_out,     # Plasma power output (little endian)
                             he_relay,    # Helium relay
                             he_out,      # Helium output (little endian)
                             ar_relay,    # Argon relay
                             ar_out)      # Argon output (little endian)
    
    # Calculate checksum (sum of first 12 bytes)
    checksum = sum(packet_data)
    
    # Add checksum (little endian)
    full_packet = packet_data + struct.pack('<I', checksum)
    
    return full_packet

def format_hex_string(packet_bytes):
    """Format packet as hex string"""
    return ' '.join(f'{b:02X}' for b in packet_bytes)

# Generate all test packets
test_packets = [
    {
        'name': 'Test Packet 1: Normal Working State',
        'desc': 'All devices working normally, plasma power output 1000, helium flow 500, argon flow 300',
        'packet': create_packet(0x00, 0x11, 1000, 0x11, 500, 0x11, 300)
    },
    {
        'name': 'Test Packet 2: Emergency Stop State',
        'desc': 'Emergency stop activated, all outputs 0',
        'packet': create_packet(0xF8, 0x00, 0, 0x00, 0, 0x00, 0)
    },
    {
        'name': 'Test Packet 3: Partial Device ON',
        'desc': 'Only plasma power and helium ON, argon OFF',
        'packet': create_packet(0x00, 0x10, 100, 0x10, 200, 0x00, 0)
    },
    {
        'name': 'Test Packet 4: High Output Value Test',
        'desc': 'All devices ON, high output value test',
        'packet': create_packet(0x00, 0x11, 4095, 0x11, 4095, 0x11, 4095)
    },
    {
        'name': 'Test Packet 5: Minimum Output Value Test',
        'desc': 'All devices ON, minimum output value',
        'packet': create_packet(0x00, 0x11, 1, 0x11, 1, 0x11, 1)
    },
    {
        'name': 'Test Packet 6: Invalid Checksum Test',
        'desc': 'Intentionally wrong checksum for testing checksum function',
        'packet': b'\xFF\xFE\x00\x11\xE8\x03\x11\xF4\x01\x11\x2C\x01\xFF\xFF\xFF\xFF'
    },
    {
        'name': 'Test Packet 7: Helium Only Test',
        'desc': 'Only helium flow meter and solenoid valve ON',
        'packet': create_packet(0x00, 0x00, 0, 0x11, 400, 0x00, 0)
    },
    {
        'name': 'Test Packet 8: Argon Only Test',
        'desc': 'Only argon flow meter and solenoid valve ON',
        'packet': create_packet(0x00, 0x00, 0, 0x00, 0, 0x11, 900)
    },
    {
        'name': 'Test Packet 9: Boundary Value Test',
        'desc': '16-bit value boundary test (65535)',
        'packet': create_packet(0x00, 0x11, 65535, 0x11, 65535, 0x11, 65535)
    },
    {
        'name': 'Test Packet 10: Mixed State Test',
        'desc': 'Plasma power OFF, helium flow meter ON, argon solenoid valve ON',
        'packet': create_packet(0x00, 0x01, 50, 0x10, 300, 0x01, 600)
    }
]

print("Generated Test Packets:")
print("=" * 60)

for i, test in enumerate(test_packets, 1):
    print(f"## {test['name']}")
    print(f"# {test['desc']}")
    print(format_hex_string(test['packet']))
    print()