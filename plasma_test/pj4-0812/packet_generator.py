#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Plasma Control System Packet Generator
Used to generate and verify test packets
"""

import struct

class PlasmaPacketGenerator:
    """Plasma Control System Packet Generator"""
    
    HEADER = 0xFEFF
    PACKET_SIZE = 16
    
    def __init__(self):
        pass
    
    def create_packet(self, emer_stop=0x00, vol_relay=0x00, vol_out=0, 
                     he_relay=0x00, he_out=0, ar_relay=0x00, ar_out=0):
        """
        Create data packet
        
        Parameters:
        - emer_stop: Emergency stop flag (0x00=normal, 0xF8=emergency stop)
        - vol_relay: Plasma power relay status (high 4 bits=plasma power, low 4 bits=voltage regulator)
        - vol_out: Plasma power output value (0-65535)
        - he_relay: Helium flow relay status (high 4 bits=flow meter, low 4 bits=solenoid valve)
        - he_out: Helium output value (0-65535)
        - ar_relay: Argon flow relay status (high 4 bits=flow meter, low 4 bits=solenoid valve)
        - ar_out: Argon output value (0-65535)
        
        Returns:
        - bytes: 16-byte data packet
        """
        
        # Build first 12 bytes of data
        packet_data = struct.pack('<HBBHBHBH', 
                                 self.HEADER,    # Header (little endian)
                                 emer_stop,      # Emergency stop
                                 vol_relay,      # Plasma power relay
                                 vol_out,        # Plasma power output (little endian)
                                 he_relay,       # Helium relay
                                 he_out,         # Helium output (little endian)
                                 ar_relay,       # Argon relay
                                 ar_out)         # Argon output (little endian)
        
        # Calculate checksum (sum of first 12 bytes)
        checksum = sum(packet_data)
        
        # Add checksum (little endian)
        full_packet = packet_data + struct.pack('<I', checksum)
        
        return full_packet
    
    def parse_packet(self, packet_bytes):
        """
        Parse data packet
        
        Parameters:
        - packet_bytes: 16-byte data packet
        
        Returns:
        - dict: Parsed data dictionary
        """
        
        if len(packet_bytes) != self.PACKET_SIZE:
            raise ValueError(f"Packet length error, expected {self.PACKET_SIZE} bytes, got {len(packet_bytes)} bytes")
        
        # Parse first 12 bytes
        header, emer_stop, vol_relay, vol_out, he_relay, he_out, ar_relay, ar_out = \
            struct.unpack('<HBBHBHBH', packet_bytes[:12])
        
        # Parse checksum
        checksum = struct.unpack('<I', packet_bytes[12:16])[0]
        
        # Verify header
        if header != self.HEADER:
            raise ValueError(f"Header error, expected 0x{self.HEADER:04X}, got 0x{header:04X}")
        
        # Verify checksum
        calculated_checksum = sum(packet_bytes[:12])
        checksum_valid = (checksum == calculated_checksum)
        
        return {
            'header': header,
            'emer_stop': emer_stop,
            'vol_relay': vol_relay,
            'vol_out': vol_out,
            'he_relay': he_relay,
            'he_out': he_out,
            'ar_relay': ar_relay,
            'ar_out': ar_out,
            'checksum': checksum,
            'calculated_checksum': calculated_checksum,
            'checksum_valid': checksum_valid
        }
    
    def format_hex_string(self, packet_bytes):
        """
        Format packet as hex string
        
        Parameters:
        - packet_bytes: packet bytes
        
        Returns:
        - str: formatted hex string
        """
        return ' '.join(f'{b:02X}' for b in packet_bytes)
    
    def relay_state_to_string(self, relay_byte):
        """
        Convert relay status byte to readable string
        
        Parameters:
        - relay_byte: relay status byte
        
        Returns:
        - str: readable status description
        """
        high_bit = (relay_byte & 0xF0) >> 4
        low_bit = relay_byte & 0x0F
        
        high_state = "ON" if high_bit else "OFF"
        low_state = "ON" if low_bit else "OFF"
        
        return f"High 4 bits: {high_state}, Low 4 bits: {low_state}"

def main():
    """Main function - demonstration"""
    
    generator = PlasmaPacketGenerator()
    
    print("=== STM32 Plasma Control System Packet Generator ===")
    print()
    
    # Generate test packets
    test_cases = [
        {
            'name': 'Normal Working State',
            'params': {
                'emer_stop': 0x00,
                'vol_relay': 0x11,  # Both plasma power and voltage regulator ON
                'vol_out': 1000,
                'he_relay': 0x11,   # Both helium flow meter and solenoid valve ON
                'he_out': 500,
                'ar_relay': 0x11,   # Both argon flow meter and solenoid valve ON
                'ar_out': 300
            }
        },
        {
            'name': 'Emergency Stop State',
            'params': {
                'emer_stop': 0xF8,
                'vol_relay': 0x00,
                'vol_out': 0,
                'he_relay': 0x00,
                'he_out': 0,
                'ar_relay': 0x00,
                'ar_out': 0
            }
        },
        {
            'name': 'Partial Device ON',
            'params': {
                'emer_stop': 0x00,
                'vol_relay': 0x10,  # Only plasma power ON
                'vol_out': 100,
                'he_relay': 0x10,   # Only helium flow meter ON
                'he_out': 200,
                'ar_relay': 0x00,   # Argon all OFF
                'ar_out': 0
            }
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"Test Packet {i}: {test_case['name']}")
        
        # Generate packet
        packet = generator.create_packet(**test_case['params'])
        
        # Format output
        hex_string = generator.format_hex_string(packet)
        print(f"Hex: {hex_string}")
        
        # Parse and verify
        try:
            parsed = generator.parse_packet(packet)
            print(f"Parse result:")
            print(f"  Emergency Stop: {'YES' if parsed['emer_stop'] == 0xF8 else 'NO'}")
            print(f"  Plasma Power Relay: {generator.relay_state_to_string(parsed['vol_relay'])}")
            print(f"  Plasma Power Output: {parsed['vol_out']}")
            print(f"  Helium Relay: {generator.relay_state_to_string(parsed['he_relay'])}")
            print(f"  Helium Output: {parsed['he_out']}")
            print(f"  Argon Relay: {generator.relay_state_to_string(parsed['ar_relay'])}")
            print(f"  Argon Output: {parsed['ar_out']}")
            print(f"  Checksum: {'VALID' if parsed['checksum_valid'] else 'INVALID'} ({parsed['checksum']})")
        except Exception as e:
            print(f"Parse error: {e}")
        
        print("-" * 50)
    
    print("\nPacket generation completed!")
    print("Check test_packets.txt for more test data.")

if __name__ == '__main__':
    main()