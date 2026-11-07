"""Decode packed RC channel frames from a TBS Crossfire serial receiver."""

CRSF_RC_CHANNELS_PACKED = 0x16
CRSF_FLIGHT_CONTROLLER_ADDRESS = 0xC8
CRSF_RC_CHANNEL_COUNT = 16
CRSF_RC_PAYLOAD_SIZE = 22
CRSF_RC_FRAME_LENGTH = 24
CRSF_MAX_FRAME_LENGTH = 62
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811


def crsf_crc8(data: bytes) -> int:
    """Calculate the CRSF CRC-8/DVB-S2 checksum."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def decode_rc_channels(payload: bytes) -> tuple[float, ...]:
    """Decode 16 packed 11-bit channels and normalize them to [-1, 1]."""
    if len(payload) != CRSF_RC_PAYLOAD_SIZE:
        raise ValueError(
            f'Le payload RC doit contenir {CRSF_RC_PAYLOAD_SIZE} octets.'
        )

    packed_channels = int.from_bytes(payload, byteorder='little')
    channels: list[float] = []
    for index in range(CRSF_RC_CHANNEL_COUNT):
        raw_value = (packed_channels >> (11 * index)) & 0x7FF
        raw_value = max(
            CRSF_CHANNEL_MIN,
            min(raw_value, CRSF_CHANNEL_MAX),
        )
        if raw_value >= CRSF_CHANNEL_CENTER:
            normalized = (
                (raw_value - CRSF_CHANNEL_CENTER)
                / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTER)
            )
        else:
            normalized = (
                (raw_value - CRSF_CHANNEL_CENTER)
                / (CRSF_CHANNEL_CENTER - CRSF_CHANNEL_MIN)
            )
        channels.append(normalized)
    return tuple(channels)


class CrsfFrameParser:
    """Extract valid RC channel frames from an arbitrary serial byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[float, ...]]:
        """Return decoded channel sets from all complete valid frames."""
        self.buffer.extend(data)
        channel_sets: list[tuple[float, ...]] = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != CRSF_FLIGHT_CONTROLLER_ADDRESS:
                del self.buffer[0]
                continue

            frame_length = self.buffer[1]
            if frame_length < 2 or frame_length > CRSF_MAX_FRAME_LENGTH:
                del self.buffer[0]
                continue

            total_length = frame_length + 2
            if len(self.buffer) < total_length:
                break

            frame = bytes(self.buffer[:total_length])
            if crsf_crc8(frame[2:-1]) != frame[-1]:
                del self.buffer[0]
                continue

            del self.buffer[:total_length]
            if (
                frame[2] == CRSF_RC_CHANNELS_PACKED
                and frame_length == CRSF_RC_FRAME_LENGTH
            ):
                channel_sets.append(decode_rc_channels(frame[3:-1]))
        return channel_sets
